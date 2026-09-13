#!/usr/bin/env python3
"""
Noeud ROS2 de generation de trafic pour l'environnement Active SLAM.

Peuple l'environnement CARLA avec des vehicules ET des pietons NPC
(necessaires pour que le critere 'safety' de decision_maker ait
reellement des obstacles a evaluer), et les garde en mouvement :
  - vehicules  : pilote automatique via le Traffic Manager, qui
    respecte nativement les feux, les stops, les distances de
    securite et les limites de vitesse ;
  - pietons    : IA de marche CARLA (controller.ai.walker), avec
    une destination aleatoire sur le maillage de navigation.

Le trafic doit SUIVRE l'ego (correction importante)
---------------------------------------------------
La version precedente spawnait le trafic une seule fois, dans un
rayon de spawn_radius_from_ego autour de la position de DEPART, puis
maintenait la population en NOMBRE : elle ne recompletait que si un
vehicule avait ete detruit. Or les NPC ne meurent quasiment jamais.

Consequence observee : l'ego parcourt plusieurs centaines de metres,
les NPC restent groupes autour du point de depart, le journal affiche
fidelement "8 vehicules" alors qu'il n'y en a plus un seul dans le
champ des capteurs. Le critere 'safety' n'a plus rien a evaluer, et
le scenario "trafic" exige par le sujet n'existe que pendant la
premiere minute du run.

Le watchdog compte desormais les vehicules reellement PRES de l'ego
(active_radius). Quand ce nombre passe sous min_vehicles_near_ego, il
RECYCLE les vehicules devenus trop lointains (au-dela de
recycle_beyond) en les repositionnant dans une bande devant l'ego
plutot que d'en spawner de nouveaux : la charge du simulateur reste
constante.

Les vehicules recycles ne sont jamais deposes a moins de
recycle_min_distance : un vehicule qui se materialise dans le champ
du LiDAR cree un obstacle fantome dans le nuage de points et pollue
la carte SLAM.

Montee en charge progressive (ramp)
-----------------------------------
ramp_enabled fait croitre num_vehicles de sa valeur initiale vers
ramp_final_vehicles sur ramp_duration secondes.

ATTENTION METHODOLOGIQUE : la rampe est indexee sur le TEMPS. Deux
strategies qui parcourent le meme budget de distance ne mettent pas
la meme duree (feux rouges, detours), donc elles ne rencontrent pas
la meme densite de trafic. Utilisee pendant la campagne comparative,
la rampe introduit donc un biais qui favorise arbitrairement les
strategies rapides. Elle doit rester DESACTIVEE pour la comparaison
des strategies, et n'etre activee que dans un scenario dedie de
robustesse a la densite, ou la montee en charge est justement l'objet
de la mesure.

Densite et rayon de spawn
-------------------------
Town03 compte environ 265 points de spawn repartis sur toute la
carte. Un trafic de quelques vehicules disperses au hasard sur cette
surface est statistiquement invisible depuis le vehicule ego. Deux
parametres repondent a cela : num_vehicles / num_walkers pour la
densite globale, spawn_radius_from_ego pour concentrer le spawn la ou
il sera effectivement rencontre.

Attente du vehicule ego avant tout spawn (IMPORTANT)
----------------------------------------------------
Tous les noeuds d'un launch ROS2 demarrent SIMULTANEMENT. Or
bridge_active_slam charge la carte au demarrage, et un chargement de
monde CARLA DETRUIT tous les acteurs presents. Ce noeud attend donc
l'apparition du vehicule ego avant de spawner.

Mode synchrone
--------------
bridge_active_slam configure CARLA en synchronous_mode=True et fait
avancer la simulation via world.tick() -- lui seul doit le faire. Ce
noeud n'appelle donc JAMAIS world.tick() ni world.apply_settings(). Il
lit le mode reel du monde et aligne le Traffic Manager dessus.

Ce noeud ne touche jamais au vehicule ego, gere separement par
vehicle_controller_active_slam.py.
"""

import math
import random
import time

import carla

import rclpy
from rclpy.node import Node


class TrafficSpawnerNode(Node):

    def __init__(self):
        super().__init__('active_slam_traffic_spawner')

        # --- Parametres (noms alignes sur active_slam.launch.py) ---
        self.declare_parameter('carla_host', '127.0.0.1')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('ego_role_name', 'ego')
        self.declare_parameter('num_vehicles', 40)
        self.declare_parameter('num_walkers', 15)
        self.declare_parameter('min_distance_from_ego', 10.0)
        # Rayon max de spawn autour de l'ego (0 = toute la carte).
        self.declare_parameter('spawn_radius_from_ego', 150.0)

        # --- Suivi de l'ego ---
        # active_radius : rayon dans lequel un vehicule compte comme
        # "reellement rencontre" par l'ego et percu par ses capteurs.
        self.declare_parameter('active_radius', 120.0)
        # Nombre de vehicules a maintenir dans ce rayon.
        # 0 desactive completement le suivi (comportement d'origine).
        self.declare_parameter('min_vehicles_near_ego', 5)
        # Un vehicule n'est recycle que s'il est plus loin que ceci.
        self.declare_parameter('recycle_beyond', 250.0)
        # Bande de depot des vehicules recycles. Le minimum protege le
        # nuage LiDAR d'un obstacle qui se materialise dans son champ.
        self.declare_parameter('recycle_min_distance', 40.0)
        self.declare_parameter('recycle_max_distance', 140.0)
        # Borne par cycle de watchdog : un repositionnement est un appel
        # reseau, et on ne veut pas figer la boucle.
        self.declare_parameter('max_recycles_per_tick', 2)

        # --- Montee en charge progressive (voir docstring) ---
        self.declare_parameter('ramp_enabled', False)
        self.declare_parameter('ramp_final_vehicles', 0)
        self.declare_parameter('ramp_duration', 0.0)

        self.declare_parameter('wait_for_ego_timeout', 90.0)
        self.declare_parameter('spawn_settle_delay', 2.0)

        self.declare_parameter('tm_port', 8000)
        self.declare_parameter('seed', -1)                      # -1 = aleatoire
        self.declare_parameter('speed_diff_percent', -10.0)     # negatif = plus rapide que la limite
        self.declare_parameter('distance_to_leading', 2.5)      # m
        self.declare_parameter('ignore_lights_percent', 0.0)    # 0 = respect strict des feux
        self.declare_parameter('ignore_signs_percent', 0.0)     # 0 = respect strict des stops
        self.declare_parameter('walker_speed', 1.4)             # m/s, marche normale

        self.declare_parameter('watchdog_rate', 1.0)            # Hz
        self.declare_parameter('stuck_speed_threshold', 0.3)    # m/s
        self.declare_parameter('stuck_time_limit', 12.0)        # s hors feu rouge avant intervention
        self.declare_parameter('maintain_population', True)
        self.declare_parameter('status_period', 10.0)           # s entre deux resumes

        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        self.ego_role_name = self.get_parameter('ego_role_name').value
        self.num_vehicles = self.get_parameter('num_vehicles').value
        self.num_walkers = self.get_parameter('num_walkers').value
        self.min_distance_from_ego = self.get_parameter('min_distance_from_ego').value
        self.spawn_radius_from_ego = self.get_parameter('spawn_radius_from_ego').value

        self.active_radius = float(self.get_parameter('active_radius').value)
        self.min_vehicles_near_ego = int(
            self.get_parameter('min_vehicles_near_ego').value
        )
        self.recycle_beyond = float(self.get_parameter('recycle_beyond').value)
        self.recycle_min_distance = float(
            self.get_parameter('recycle_min_distance').value
        )
        self.recycle_max_distance = float(
            self.get_parameter('recycle_max_distance').value
        )
        self.max_recycles_per_tick = int(
            self.get_parameter('max_recycles_per_tick').value
        )

        self.ramp_enabled = bool(self.get_parameter('ramp_enabled').value)
        self.ramp_final_vehicles = int(
            self.get_parameter('ramp_final_vehicles').value
        )
        self.ramp_duration = float(self.get_parameter('ramp_duration').value)
        self.initial_num_vehicles = self.num_vehicles
        self.ramp_start_time = None

        self.wait_for_ego_timeout = self.get_parameter('wait_for_ego_timeout').value
        self.spawn_settle_delay = self.get_parameter('spawn_settle_delay').value

        self.tm_port = self.get_parameter('tm_port').value
        seed = self.get_parameter('seed').value
        self.speed_diff_percent = self.get_parameter('speed_diff_percent').value
        self.distance_to_leading = self.get_parameter('distance_to_leading').value
        self.ignore_lights_percent = self.get_parameter('ignore_lights_percent').value
        self.ignore_signs_percent = self.get_parameter('ignore_signs_percent').value
        self.walker_speed = self.get_parameter('walker_speed').value

        self.watchdog_period = 1.0 / self.get_parameter('watchdog_rate').value
        self.stuck_speed_threshold = self.get_parameter('stuck_speed_threshold').value
        self.stuck_time_limit = self.get_parameter('stuck_time_limit').value
        self.maintain_population = self.get_parameter('maintain_population').value
        self.status_period = self.get_parameter('status_period').value

        # --- Etat ---
        self.npc_vehicles = []
        self.walkers = []
        self.walker_controllers = []
        self.stuck_since = {}          # {actor_id: timestamp premiere detection}
        self.total_recycled = 0

        self.carla_map = None
        self.all_spawn_points = []     # cache : get_map() est couteux
        self.traffic_manager = None
        self.traffic_ready = False
        self.startup_attempts = 0
        self.startup_start_time = time.time()
        self.watchdog_timer = None
        self.last_status_time = 0.0

        # --- Connexion CARLA (avec retry) ---
        self.world = None
        max_attempts = 6
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(
                f"Connexion a CARLA {host}:{port} (essai {attempt}/{max_attempts}) ..."
            )
            try:
                self.client = carla.Client(host, port)
                self.client.set_timeout(20.0)
                self.world = self.client.get_world()
                break
            except RuntimeError as e:
                self.get_logger().warn(f"Echec de connexion : {e}")
                if attempt == max_attempts:
                    self.get_logger().error(
                        "Impossible de se connecter a CARLA apres plusieurs essais, arret du noeud"
                    )
                    raise
                time.sleep(5.0)

        if seed is not None and seed >= 0:
            random.seed(seed)
        self.seed = seed

        # Le spawn n'a PAS lieu ici : il attend que le bridge ait charge la
        # carte et fait apparaitre l'ego (voir docstring).
        self.startup_timer = self.create_timer(1.0, self._startup_tick)

        self.get_logger().info(
            "traffic_spawner demarre, attente du chargement de la carte "
            f"et du vehicule ego (role_name='{self.ego_role_name}')..."
        )

    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------

    def _find_ego(self):
        """Retourne l'acteur ego, ou None."""
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.ego_role_name:
                return actor
        return None

    def _ego_location(self):
        ego = self._find_ego()
        return ego.get_location() if ego is not None else None

    def _wait_for_actors(self, actor_ids, timeout=5.0):
        """Attend que les acteurs demandes soient visibles dans le monde.

        En mode synchrone, apply_batch_sync(..., False) ne fait pas
        avancer la simulation : les acteurs n'apparaissent qu'au tick
        suivant, emis par bridge_active_slam. On sonde donc le monde
        plutot que de ticker nous-memes.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            actors = [self.world.get_actor(aid) for aid in actor_ids]
            if all(a is not None for a in actors):
                return actors
            time.sleep(0.05)
        return [self.world.get_actor(aid) for aid in actor_ids]

    @staticmethod
    def _alive(actors):
        out = []
        for a in actors:
            if a is None:
                continue
            try:
                if a.is_alive:
                    out.append(a)
            except RuntimeError:
                continue
        return out

    @staticmethod
    def _speed(actor):
        v = actor.get_velocity()
        return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

    @staticmethod
    def _at_red_light(vehicle):
        try:
            return (
                vehicle.is_at_traffic_light()
                and vehicle.get_traffic_light_state() == carla.TrafficLightState.Red
            )
        except RuntimeError:
            return False

    # ------------------------------------------------------------------
    # Sequence de demarrage
    # ------------------------------------------------------------------

    def _startup_tick(self):
        """Attend l'ego (donc la fin du chargement de carte), puis spawn."""
        self.startup_attempts += 1

        try:
            self.world = self.client.get_world()
        except RuntimeError as e:
            self.get_logger().warn(f"Monde CARLA indisponible : {e}")
            return

        ego = self._find_ego()
        waited = time.time() - self.startup_start_time

        if ego is None:
            if waited < self.wait_for_ego_timeout:
                if self.startup_attempts % 10 == 0:
                    self.get_logger().info(
                        f"Vehicule ego pas encore present ({waited:.0f}s), "
                        f"spawn du trafic differe."
                    )
                return
            self.get_logger().warn(
                f"Vehicule ego toujours absent apres {waited:.0f}s : "
                f"generation du trafic sans reference a l'ego."
            )

        if self.spawn_settle_delay > 0:
            time.sleep(self.spawn_settle_delay)
            self.world = self.client.get_world()

        self.carla_map = self.world.get_map()
        self.all_spawn_points = self.carla_map.get_spawn_points()

        settings = self.world.get_settings()
        self.get_logger().info(
            f"Carte '{self.carla_map.name}' prete "
            f"({len(self.all_spawn_points)} points de spawn, "
            f"synchronous_mode={settings.synchronous_mode}), generation du trafic..."
        )

        self._configure_traffic_manager(settings.synchronous_mode)

        ego_location = self._ego_location()
        self._spawn_vehicles(ego_location)
        self._spawn_walkers(ego_location)

        if len(self.npc_vehicles) + len(self.walkers) == 0:
            self.get_logger().warn(
                "Aucun acteur NPC genere lors de cet essai, nouvelle tentative..."
            )
            return  # le timer de demarrage reste actif et reessaiera

        self.startup_timer.cancel()
        self.traffic_ready = True
        self.last_status_time = time.time()
        self.ramp_start_time = time.time()

        self.watchdog_timer = self.create_timer(self.watchdog_period, self._watchdog_tick)

        radius_txt = (
            f"dans un rayon de {self.spawn_radius_from_ego:.0f}m autour de l'ego"
            if self.spawn_radius_from_ego > 0 else "sur toute la carte"
        )
        suivi_txt = (
            f"suivi actif : au moins {self.min_vehicles_near_ego} vehicules "
            f"maintenus a moins de {self.active_radius:.0f}m de l'ego"
            if self.min_vehicles_near_ego > 0
            else "suivi de l'ego DESACTIVE (le trafic restera au point de depart)"
        )
        self.get_logger().info(
            f"TRAFIC ACTIF : {len(self.npc_vehicles)} vehicules en pilote automatique "
            f"(respect des feux et des stops) et {len(self.walkers)} pietons, {radius_txt}. "
            f"{suivi_txt}."
        )

        if self.ramp_enabled and self.ramp_final_vehicles > self.num_vehicles:
            self.get_logger().info(
                f"MONTEE EN CHARGE : {self.initial_num_vehicles} -> "
                f"{self.ramp_final_vehicles} vehicules sur {self.ramp_duration:.0f}s. "
                f"A n'utiliser QUE dans un scenario dedie : indexee sur le temps, "
                f"elle biaise la comparaison entre strategies."
            )

    def _configure_traffic_manager(self, world_is_synchronous):
        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)

        # Alignement sur le mode REEL du monde : un TM asynchrone dans un
        # monde synchrone laisse tous les vehicules immobiles.
        self.traffic_manager.set_synchronous_mode(world_is_synchronous)

        if self.seed is not None and self.seed >= 0:
            self.traffic_manager.set_random_device_seed(self.seed)

        self.traffic_manager.set_global_distance_to_leading_vehicle(self.distance_to_leading)
        self.traffic_manager.global_percentage_speed_difference(self.speed_diff_percent)

        try:
            self.traffic_manager.set_respawn_dormant_vehicles(True)
        except AttributeError:
            pass  # option absente selon la version de CARLA

    # ------------------------------------------------------------------
    # Selection des points de spawn
    # ------------------------------------------------------------------

    def _candidate_spawn_points(self, ego_location, dmin=None, dmax=None):
        """Points de spawn libres, dans une bande de distance autour de l'ego.

        dmin / dmax permettent aux repositionnements d'utiliser une bande
        differente de celle du spawn initial. Si la bande ne laisse pas
        assez de points, on elargit plutot que de ne rien renvoyer : mieux
        vaut du trafic loin que pas de trafic du tout.
        """
        if not self.all_spawn_points:
            return []

        if dmin is None:
            dmin = self.min_distance_from_ego
        if dmax is None:
            dmax = (
                self.spawn_radius_from_ego
                if self.spawn_radius_from_ego > 0 else float('inf')
            )

        points = list(self.all_spawn_points)

        if ego_location is not None:
            near = [
                sp for sp in points
                if dmin < sp.location.distance(ego_location) < dmax
            ]
            if len(near) >= 3:
                points = near
            else:
                points = [
                    sp for sp in points
                    if sp.location.distance(ego_location) > dmin
                ] or points

        # Points deja occupes par un vehicule existant : un spawn dessus
        # echouerait (collision) ou creerait un empilement.
        occupied = [a.get_location() for a in self.world.get_actors().filter('vehicle.*')]
        free = [
            sp for sp in points
            if all(sp.location.distance(loc) > 6.0 for loc in occupied)
        ]

        random.shuffle(free)
        return free

    # ------------------------------------------------------------------
    # Spawn des vehicules
    # ------------------------------------------------------------------

    def _spawn_vehicles(self, ego_location, count=None, wait_timeout=5.0):
        if count is None:
            count = self.num_vehicles
        if count <= 0:
            return 0

        blueprint_library = self.world.get_blueprint_library()
        excluded_keywords = ('bike', 'motorcycle')
        vehicle_blueprints = [
            bp for bp in blueprint_library.filter('vehicle.*')
            if not any(kw in bp.id.lower() for kw in excluded_keywords)
        ]
        if not vehicle_blueprints:
            self.get_logger().error("Aucun blueprint de vehicule disponible.")
            return 0

        spawn_points = self._candidate_spawn_points(ego_location)
        if not spawn_points:
            return 0

        n = min(count, len(spawn_points))

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        for i in range(n):
            blueprint = random.choice(vehicle_blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            batch.append(
                SpawnActor(blueprint, spawn_points[i]).then(
                    SetAutopilot(FutureActor, True, self.traffic_manager.get_port())
                )
            )

        spawned_ids = []
        failures = 0
        for response in self.client.apply_batch_sync(batch, False):
            if response.error:
                failures += 1
                if failures <= 3:   # on ne noie pas les logs
                    self.get_logger().warn(f"Spawn vehicule echoue : {response.error}")
            else:
                spawned_ids.append(response.actor_id)

        new_vehicles = self._alive(self._wait_for_actors(spawned_ids, timeout=wait_timeout))

        for vehicle in new_vehicles:
            try:
                # Filet de securite : si le .then(SetAutopilot) n'a pas ete
                # applique, un vehicule sans autopilot resterait immobile.
                vehicle.set_autopilot(True, self.traffic_manager.get_port())
                self.traffic_manager.ignore_lights_percentage(vehicle, self.ignore_lights_percent)
                self.traffic_manager.ignore_signs_percentage(vehicle, self.ignore_signs_percent)
                self.traffic_manager.auto_lane_change(vehicle, True)
            except RuntimeError as e:
                self.get_logger().warn(f"Configuration autopilot echouee : {e}")

        self.npc_vehicles.extend(new_vehicles)

        if failures or len(new_vehicles) < n:
            self.get_logger().warn(
                f"{len(new_vehicles)}/{n} vehicules effectivement presents "
                f"({failures} spawns refuses par le simulateur)."
            )

        return len(new_vehicles)

    # ------------------------------------------------------------------
    # Spawn des pietons
    # ------------------------------------------------------------------

    def _spawn_walkers(self, ego_location):
        if self.num_walkers <= 0:
            return

        blueprint_library = self.world.get_blueprint_library()
        walker_blueprints = list(blueprint_library.filter('walker.pedestrian.*'))
        if not walker_blueprints:
            self.get_logger().warn("Aucun blueprint pieton disponible sur cette carte.")
            return

        spawn_locations = []
        attempts = 0
        max_attempts = self.num_walkers * 40
        while len(spawn_locations) < self.num_walkers and attempts < max_attempts:
            attempts += 1
            loc = self.world.get_random_location_from_navigation()
            if loc is None:
                continue
            if ego_location is not None:
                d = loc.distance(ego_location)
                if d < self.min_distance_from_ego:
                    continue
                if 0 < self.spawn_radius_from_ego < d:
                    continue
            spawn_locations.append(loc)

        if not spawn_locations:
            self.get_logger().warn(
                "Aucune position de navigation valide trouvee pour les pietons. "
                "Le maillage de navigation pietonne est probablement absent de "
                "cette carte (variantes _Opt)."
            )
            return

        SpawnActor = carla.command.SpawnActor

        # --- Etape 1 : corps des pietons ---
        batch = []
        for loc in spawn_locations:
            blueprint = random.choice(walker_blueprints)
            if blueprint.has_attribute('is_invincible'):
                blueprint.set_attribute('is_invincible', 'false')
            batch.append(SpawnActor(blueprint, carla.Transform(loc)))

        walker_ids = []
        for response in self.client.apply_batch_sync(batch, False):
            if not response.error:
                walker_ids.append(response.actor_id)

        if not walker_ids:
            self.get_logger().warn("Aucun pieton n'a pu etre cree.")
            return

        self._wait_for_actors(walker_ids)

        # --- Etape 2 : controleurs IA attaches ---
        walker_controller_bp = blueprint_library.find('controller.ai.walker')
        batch = [
            SpawnActor(walker_controller_bp, carla.Transform(), wid)
            for wid in walker_ids
        ]

        controller_ids = []
        for response in self.client.apply_batch_sync(batch, False):
            if not response.error:
                controller_ids.append(response.actor_id)

        self.walkers = self._alive(self._wait_for_actors(walker_ids))
        self.walker_controllers = self._alive(self._wait_for_actors(controller_ids))

        for controller in self.walker_controllers:
            try:
                controller.start()
                target = self.world.get_random_location_from_navigation()
                if target is not None:
                    controller.go_to_location(target)
                controller.set_max_speed(self.walker_speed)
            except RuntimeError as e:
                self.get_logger().warn(f"Demarrage controleur pieton echoue : {e}")

    # ------------------------------------------------------------------
    # Montee en charge progressive
    # ------------------------------------------------------------------

    def _update_ramp(self):
        """Fait croitre lineairement la cible num_vehicles au cours du run.

        Desactivee par defaut : voir l'avertissement methodologique dans
        la docstring du module.
        """
        if not self.ramp_enabled:
            return
        if self.ramp_duration <= 0 or self.ramp_final_vehicles <= self.initial_num_vehicles:
            return
        if self.ramp_start_time is None:
            return

        ecoule = time.time() - self.ramp_start_time
        avancement = min(1.0, ecoule / self.ramp_duration)

        cible = int(round(
            self.initial_num_vehicles
            + avancement * (self.ramp_final_vehicles - self.initial_num_vehicles)
        ))

        if cible != self.num_vehicles:
            self.num_vehicles = cible

    # ------------------------------------------------------------------
    # Suivi de l'ego par recyclage
    # ------------------------------------------------------------------

    def _follow_ego(self, ego_location):
        """Ramene les vehicules trop lointains dans le voisinage de l'ego.

        On RECYCLE plutot que de spawner : le nombre total d'acteurs reste
        constant, donc la charge du simulateur aussi. Spawner sans detruire
        ferait grimper la population a chaque cycle et finirait par saturer
        le GPU.

        Retourne (nb_proches_avant, nb_recycles).
        """
        if self.min_vehicles_near_ego <= 0 or ego_location is None:
            return (0, 0)

        distances = []
        for vehicle in self.npc_vehicles:
            try:
                distances.append(
                    (vehicle.get_location().distance(ego_location), vehicle)
                )
            except RuntimeError:
                continue

        proches = sum(1 for d, _ in distances if d <= self.active_radius)

        manquants = self.min_vehicles_near_ego - proches
        if manquants <= 0:
            return (proches, 0)

        # Candidats au recyclage : les plus lointains d'abord.
        lointains = sorted(
            (item for item in distances if item[0] > self.recycle_beyond),
            key=lambda item: item[0],
            reverse=True,
        )

        if not lointains:
            return (proches, 0)

        points = self._candidate_spawn_points(
            ego_location,
            dmin=self.recycle_min_distance,
            dmax=self.recycle_max_distance,
        )
        if not points:
            return (proches, 0)

        a_recycler = min(manquants, len(lointains), len(points),
                         self.max_recycles_per_tick)

        recycles = 0
        for i in range(a_recycler):
            _, vehicle = lointains[i]
            try:
                vehicle.set_transform(points[i])
                vehicle.set_autopilot(True, self.traffic_manager.get_port())
                recycles += 1
            except RuntimeError as e:
                self.get_logger().warn(f"Recyclage echoue : {e}")

        if recycles:
            self.total_recycled += recycles
            self.get_logger().info(
                f"SUIVI EGO : {recycles} vehicule(s) ramene(s) pres de l'ego "
                f"({proches} etaient a moins de {self.active_radius:.0f}m, "
                f"cible {self.min_vehicles_near_ego}) | total recycle="
                f"{self.total_recycled}"
            )

        return (proches, recycles)

    # ------------------------------------------------------------------
    # Watchdog : anti-blocage, maintien de population, journal d'etat
    # ------------------------------------------------------------------

    def _watchdog_tick(self):
        now = time.time()

        self.npc_vehicles = self._alive(self.npc_vehicles)
        self.walkers = self._alive(self.walkers)
        self.walker_controllers = self._alive(self.walker_controllers)

        ego_location = self._ego_location()

        self._update_ramp()

        moving = 0
        at_red = 0
        stopped = 0

        for vehicle in self.npc_vehicles:
            try:
                speed = self._speed(vehicle)
            except RuntimeError:
                continue

            if speed >= self.stuck_speed_threshold:
                moving += 1
                self.stuck_since.pop(vehicle.id, None)
                continue

            if self._at_red_light(vehicle):
                # Seule immobilite legitime : on ne touche a rien.
                at_red += 1
                self.stuck_since.pop(vehicle.id, None)
                continue

            stopped += 1
            first_seen = self.stuck_since.setdefault(vehicle.id, now)
            if now - first_seen > self.stuck_time_limit:
                self._relocate(vehicle, ego_location)
                self.stuck_since[vehicle.id] = now

        for walker, controller in zip(self.walkers, self.walker_controllers):
            try:
                speed = self._speed(walker)
            except RuntimeError:
                continue

            if speed >= self.stuck_speed_threshold:
                self.stuck_since.pop(walker.id, None)
                continue

            first_seen = self.stuck_since.setdefault(walker.id, now)
            if now - first_seen > self.stuck_time_limit:
                target = self.world.get_random_location_from_navigation()
                if target is not None:
                    try:
                        controller.go_to_location(target)
                    except RuntimeError:
                        pass
                self.stuck_since[walker.id] = now

        # Suivi de l'ego : c'est ce qui fait durer le trafic tout le run.
        proches, _ = self._follow_ego(ego_location)

        # Recompletion : densite constante malgre collisions, destructions,
        # et montee en charge progressive si elle est activee.
        if self.maintain_population:
            missing = self.num_vehicles - len(self.npc_vehicles)
            if missing > 0:
                self._spawn_vehicles(ego_location, count=missing, wait_timeout=1.0)

        # Journal d'etat periodique. 'proches' est la mesure qui compte
        # reellement : un run peut afficher 30 vehicules dont aucun dans le
        # champ des capteurs.
        if self.status_period > 0 and now - self.last_status_time >= self.status_period:
            self.last_status_time = now
            self.get_logger().info(
                f"TRAFIC : {len(self.npc_vehicles)} vehicules "
                f"({moving} en mouvement, {at_red} arretes a un feu rouge, "
                f"{stopped} arretes hors feu) | "
                f"{proches} a moins de {self.active_radius:.0f}m de l'ego | "
                f"{len(self.walkers)} pietons"
            )

    def _relocate(self, vehicle, ego_location):
        """Repositionne un vehicule bloque sur un point de spawn libre,
        de preference dans le rayon utile autour de l'ego.
        """
        points = self._candidate_spawn_points(
            ego_location,
            dmin=self.recycle_min_distance,
            dmax=self.recycle_max_distance,
        )
        if not points:
            return
        try:
            vehicle.set_transform(points[0])
            vehicle.set_autopilot(True, self.traffic_manager.get_port())
            self.get_logger().info(
                f"Vehicule NPC {vehicle.id} immobile hors feu rouge "
                f"depuis {self.stuck_time_limit:.0f}s -> repositionne."
            )
        except RuntimeError as e:
            self.get_logger().warn(f"Repositionnement echoue : {e}")

    # ------------------------------------------------------------------
    # Nettoyage
    # ------------------------------------------------------------------

    def destroy_node(self):
        for controller in self.walker_controllers:
            try:
                controller.stop()
            except RuntimeError:
                pass

        destroy_batch = (
            [carla.command.DestroyActor(v) for v in self.npc_vehicles if v is not None]
            + [carla.command.DestroyActor(c) for c in self.walker_controllers if c is not None]
            + [carla.command.DestroyActor(w) for w in self.walkers if w is not None]
        )
        if destroy_batch:
            try:
                self.client.apply_batch(destroy_batch)
                self.get_logger().info(
                    f"{len(self.npc_vehicles)} vehicules et {len(self.walkers)} "
                    f"pietons NPC detruits."
                )
            except RuntimeError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
