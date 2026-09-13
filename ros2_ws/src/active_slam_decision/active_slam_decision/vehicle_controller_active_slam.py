#!/usr/bin/env python3
"""
Controleur vehicule pour l'Active SLAM.

Recoit la destination courante sur /active_slam/goal (choisie par
active_slam_decision_maker), planifie un itineraire sur le reseau
routier CARLA et pilote le vehicule ego pour s'y rendre, en tenant
compte des manoeuvres d'urgence recues sur /decision_maker/maneuver.
Publie /vehicle/arrived des que la destination est atteinte (ou
abandonnee), ce qui declenche le choix de la prochaine destination
cote decision_maker.

Planification d'itineraire
--------------------------
L'API CARLA waypoint.next() ne parcourt le graphe routier que vers
l'AVANT, en respectant le sens de circulation. Un parcours en largeur
sur ces waypoints donne donc exactement l'ensemble des points
reellement atteignables par le vehicule, ainsi que le plus court
chemin routier vers chacun. Cela remplace l'ancien suivi glouton, qui
se contentait de suivre la voie courante et ne consultait le but
qu'aux intersections : un but situe derriere ou sur le cote n'etait
alors jamais atteint.

Trois consequences :
  - le vehicule suit un chemin qui mene reellement au but ;
  - un but inatteignable est rejete immediatement (pas de poursuite
    inutile pendant tout le timeout) ;
  - la longueur de l'itineraire est la vraie "distance a parcourir"
    au sens du sujet, et non une distance a vol d'oiseau.

Direction : visee anticipee et lissage
--------------------------------------
Viser le waypoint courant de l'itineraire, souvent situe a 2-3 m,
fait sauter la cible a chaque waypoint valide : le cap demande change
brutalement dix fois par seconde et le vehicule zigzague. On applique
donc une visee anticipee de type pure pursuit (premier point de
l'itineraire au-dela de lookahead_distance), un gain de braquage
adouci (steer_gain_deg au lieu des 45 deg implicites d'origine) et une
limitation de la vitesse de braquage (max_steer_rate) : le volant ne
peut plus passer d'une butee a l'autre en un seul cycle de controle.

Conduite en trafic dense
------------------------
traffic_spawner.py peuple l'environnement de plusieurs dizaines de
vehicules NPC pilotes par le Traffic Manager, qui respectent
strictement les feux et les stops. Trois consequences pour ce noeud :

1. FILTRE PAR COULOIR, PAS PAR CONE. Le critere 'safety' de
   decision_maker (voir safety_evaluator.py) agit au niveau de la
   PLANIFICATION : il ecarte les destinations proches d'obstacles. Il
   ne dit rien de ce qui se trouve SUR le chemin. La verification
   tactique faite ici ne retient donc un obstacle que s'il est
   reellement dans le couloir de circulation du vehicule : on projette
   sa position dans le repere de l'ego et on rejette tout ce dont
   l'ecart lateral depasse obstacle_corridor_width / 2. Un simple cone
   angulaire capturait les vehicules en sens inverse, sur une voie
   parallele ou dans une rue perpendiculaire : en trafic dense, l'ego
   freinait en permanence et n'avancait plus.

2. RESPECT DES FEUX PAR L'EGO. Les NPC s'arretent aux feux rouges. Un
   ego qui les grillerait percuterait les vehicules arretes devant lui
   et bloquerait le carrefour. Il applique donc la meme regle.

3. TEMPORISATIONS RELEVEES. Attendre derriere une file arretee a un
   feu est un comportement normal, pas un blocage. Les chiens de garde
   de progression sont suspendus pendant une attente legitime (feu
   rouge ou obstacle devant), et leurs seuils sont dimensionnes pour
   des files de trafic reelles.

Desenlisement
-------------
obstacle_detector.py n'enumere que les acteurs CARLA de type vehicule et
pieton : la geometrie statique de la carte (murs, trottoirs, glissieres)
lui est invisible. Le vehicule peut donc venir buter contre un mur apres
une sortie de route. Le chien de garde de progression detecte bien
l'absence de progres, mais il se contente de changer de but : l'ego
reste physiquement plaque contre l'obstacle, et le run entier est perdu.
Une detection d'immobilite prolongee declenche donc une manoeuvre de
recul (parametres recovery_*), seule action capable de modifier l'etat
PHYSIQUE du vehicule. Le nombre de desenlisements est journalise : c'est
un indicateur de robustesse a reporter dans l'analyse comparative.

Topics :
  /active_slam/goal          (geometry_msgs/msg/PoseStamped)  <- entree
  /decision_maker/maneuver   (std_msgs/msg/String, JSON)      <- entree
  <obstacles_topic>          (geometry_msgs/msg/PoseArray)    <- entree
  /vehicle/arrived           (std_msgs/msg/Bool)              -> sortie
  /active_slam/goal_rejected (geometry_msgs/msg/PoseStamped)  -> sortie
"""

import json
import math
import time
from collections import deque

import carla

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String, Bool, Float32


class VehicleControllerActiveSLAM(Node):

    def __init__(self):
        super().__init__('vehicle_controller_active_slam')

        # --- Parametres ---
        self.declare_parameter('carla_host', '127.0.0.1')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('role_name', 'ego')
        self.declare_parameter('max_throttle', 0.5)
        self.declare_parameter('min_throttle', 0.35)
        self.declare_parameter('arrival_tolerance', 2.0)
        self.declare_parameter('maneuver_timeout', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('waypoint_reach_tolerance', 3.0)

        # Evitement tactique : distance de securite a maintenir avec un
        # obstacle situe devant le vehicule.
        self.declare_parameter('obstacle_stop_distance', 7.0)
        self.declare_parameter('obstacle_slow_distance', 14.0)
        self.declare_parameter('obstacles_topic', '/obstacle_detection/obstacles')

        # Largeur du couloir de circulation. Un obstacle n'est bloquant que
        # s'il est reellement sur la trajectoire, pas simplement quelque part
        # devant : une voie CARLA fait environ 3,5 m, un couloir de 3 m
        # (soit 1,5 m de part et d'autre de l'axe) retient les vehicules de
        # la meme voie et rejette ceux d'a cote et du sens inverse.
        self.declare_parameter('obstacle_corridor_width', 3.0)

        # Duree maximale d'attente derriere un obstacle avant de considerer
        # qu'il est definitif (vehicule gare) et non temporaire (feu rouge).
        # Relevee a 60 s : en trafic dense, une file derriere un feu peut
        # legitimement immobiliser l'ego bien plus de 30 secondes.
        self.declare_parameter('max_obstacle_wait', 60.0)

        # Respect des feux tricolores par l'ego, comme les NPC.
        self.declare_parameter('respect_traffic_lights', True)

        # Direction : visee anticipee et lissage du braquage.
        self.declare_parameter('lookahead_distance', 6.0)
        self.declare_parameter('steer_gain_deg', 60.0)
        self.declare_parameter('max_steer_rate', 0.15)

        # Planification d'itineraire
        self.declare_parameter('route_step', 4.0)
        self.declare_parameter('route_max_distance', 250.0)
        self.declare_parameter('route_goal_tolerance', 4.0)

        # Ecart maximal admis entre le vehicule et le waypoint sur
        # lequel get_waypoint() projette sa position. Une voie CARLA
        # fait environ 3,5 m de large : au-dela de 10 m, la
        # projection ne designe plus la chaussee sous le vehicule.
        self.declare_parameter('max_projection_error', 10.0)

        # Surveillance de progression. Seuils releves pour le trafic dense :
        # feux et files d'attente rallongent legitimement chaque trajet.
        self.declare_parameter('goal_timeout', 180.0)
        self.declare_parameter('no_progress_timeout', 45.0)
        self.declare_parameter('min_progress', 1.0)

        # Desenlisement. Le chien de garde de progression ci-dessus change
        # de but, mais ne change jamais l'etat physique du vehicule : une
        # fois plaque contre un mur, l'ego y reste jusqu'a la fin du run.
        # Seuils : 6 s d'immobilite suffisent a distinguer un blocage reel
        # d'un simple demarrage (un vehicule CARLA depasse 0,3 m/s en
        # moins d'une seconde), et 2,5 s de recul degagent le vehicule
        # sans le renvoyer loin dans le trafic.
        self.declare_parameter('recovery_stall_time', 6.0)
        self.declare_parameter('recovery_duration', 2.5)
        self.declare_parameter('recovery_speed_threshold', 0.3)
        self.declare_parameter('recovery_throttle', 0.4)
        # Nombre de desenlisements tentes sur un MEME but avant de
        # renoncer. Deux tentatives, l'une braquee a gauche l'autre
        # a droite, suffisent a etablir que le but n'est pas
        # atteignable depuis cette position. Au-dela on ne fait
        # qu'user le budget de temps du run.
        self.declare_parameter('max_recoveries_per_goal', 2)

        # --- Itineraire insuivable (vehicule a contresens) ---
        # Duree pendant laquelle le vehicule peut rester sur le premier
        # waypoint de son itineraire avec un cap oppose avant que cet
        # itineraire ne soit declare insuivable. Huit secondes laissent
        # le temps d'une manoeuvre legitime tout en bornant les
        # rotations sur place, mesurees jusqu'a l'expiration du budget.
        self.declare_parameter('wrong_way_timeout', 8.0)

        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        self.role_name = self.get_parameter('role_name').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.arrival_tolerance = self.get_parameter('arrival_tolerance').value
        self.maneuver_timeout = self.get_parameter('maneuver_timeout').value
        control_rate = self.get_parameter('control_rate').value
        self.waypoint_reach_tolerance = self.get_parameter('waypoint_reach_tolerance').value

        self.obstacle_stop_distance = self.get_parameter('obstacle_stop_distance').value
        self.obstacle_slow_distance = self.get_parameter('obstacle_slow_distance').value
        obstacles_topic = self.get_parameter('obstacles_topic').value
        self.obstacle_corridor_width = self.get_parameter('obstacle_corridor_width').value
        self.max_obstacle_wait = self.get_parameter('max_obstacle_wait').value
        self.respect_traffic_lights = self.get_parameter('respect_traffic_lights').value

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.steer_gain_rad = math.radians(self.get_parameter('steer_gain_deg').value)
        self.max_steer_rate = self.get_parameter('max_steer_rate').value

        self.route_step = self.get_parameter('route_step').value
        self.route_max_distance = self.get_parameter('route_max_distance').value
        self.route_goal_tolerance = self.get_parameter('route_goal_tolerance').value
        self.max_projection_error = self.get_parameter(
            'max_projection_error'
        ).value

        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.no_progress_timeout = self.get_parameter('no_progress_timeout').value
        self.min_progress = self.get_parameter('min_progress').value

        self.recovery_stall_time = self.get_parameter('recovery_stall_time').value
        self.recovery_duration = self.get_parameter('recovery_duration').value
        self.recovery_speed_threshold = self.get_parameter('recovery_speed_threshold').value
        self.recovery_throttle = self.get_parameter('recovery_throttle').value
        self.max_recoveries_per_goal = int(
            self.get_parameter('max_recoveries_per_goal').value
        )
        self.wrong_way_timeout = float(
            self.get_parameter('wrong_way_timeout').value
        )

        # Coherence : un plancher de throttle superieur au plafond
        # empecherait tout demarrage. On aligne plutot que de planter.
        if self.min_throttle > self.max_throttle:
            self.get_logger().warn(
                f"min_throttle ({self.min_throttle:.2f}) > max_throttle "
                f"({self.max_throttle:.2f}) : min_throttle ramene a "
                f"{self.max_throttle:.2f}."
            )
            self.min_throttle = self.max_throttle

        # --- Etat ---
        self.ego_vehicle = None
        self.search_attempts = 0
        self.current_goal = None          # (x, y) en repere CARLA
        self.has_arrived_published = False
        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None
        self.active_maneuver = None
        self.last_maneuver_time = 0.0

        self.route = []                   # liste de carla.Waypoint
        self.route_index = 0
        self.route_length = 0.0

        self.goal_start_time = 0.0
        self.best_distance_to_goal = float('inf')
        self.last_progress_time = 0.0

        self.abandoned_goals = 0
        self.rejected_goals = 0
        self.reached_goals = 0

        # Etat du desenlisement
        self.stall_start = None       # debut de l'immobilite en cours
        self.recovery_until = 0.0     # instant de fin du recul en cours
        self.recovery_steer = 0.0
        self.recovery_count = 0
        # Desenlisements tentes sur le but COURANT, remis a zero a
        # chaque nouvelle destination.
        self.goal_recovery_count = 0
        # Message d'origine du but courant, conserve pour pouvoir le
        # republier sur /active_slam/goal_rejected en cas d'abandon.
        # Il est exprime dans le repere 'map' des candidats, celui
        # qu'attend decision_maker.
        self.current_goal_msg = None

        # --- Distance reellement parcourue ---
        # Mesuree sur la position CARLA (verite terrain), pas sur la pose
        # SLAM : elle doit rester juste meme quand la localisation derive,
        # sinon le budget de run dependrait de la qualite du SLAM qu'on
        # cherche justement a mesurer.
        #
        # C'est la seule unite de budget commune a toutes les methodes
        # comparees. "15 destinations" n'a pas de sens pour une trajectoire
        # predefinie, et "1000 m" ne dit rien du nombre de decisions ; la
        # distance parcourue, elle, repond a la question que pose le sujet :
        # a budget de deplacement egal, quelle methode produit la meilleure
        # carte et la meilleure localisation ?
        self.distance_traveled = 0.0
        self.last_position = None
        self.last_distance_pub_time = 0.0

        self.current_obstacles = []       # positions (x, y) en repere CARLA
        self.last_obstacle_log_time = 0.0
        self.obstacle_wait_start = None
        self.last_pursue_log_time = 0.0

        # Direction lissee : dernier braquage applique, sert de point de
        # depart a la limitation de vitesse de braquage.
        self.last_steer = 0.0
        self.last_light_log_time = 0.0
        # Instant du debut de la situation 'premier waypoint jamais
        # atteint, cap oppose'. None tant qu'elle ne dure pas.
        self.wrong_way_start = None

        # --- Connexion CARLA (avec retry) ---
        self.world = None
        max_attempts = 6
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(
                f"Connexion a CARLA {host}:{port} (essai {attempt}/{max_attempts}) ..."
            )
            try:
                self.client = carla.Client(host, port)
                self.client.set_timeout(10.0)
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

        self.carla_map = self.world.get_map()

        # --- Abonnements ---
        self.create_subscription(PoseStamped, '/active_slam/goal', self.on_goal, 10)
        self.create_subscription(String, '/decision_maker/maneuver', self.on_maneuver, 10)
        self.create_subscription(PoseArray, obstacles_topic, self.on_obstacles, 10)

        # --- Publication ---
        self.arrived_pub = self.create_publisher(Bool, '/vehicle/arrived', 10)
        self.goal_rejected_pub = self.create_publisher(
            PoseStamped, '/active_slam/goal_rejected', 10
        )
        self.distance_pub = self.create_publisher(
            Float32, '/vehicle/distance_traveled', 10
        )

        # --- Boucle de controle ---
        period = 1.0 / control_rate
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            "vehicle_controller_active_slam demarre, recherche du vehicule ego..."
        )
        self.get_logger().info(
            f"Conduite en trafic : couloir={self.obstacle_corridor_width:.1f}m, "
            f"arret a {self.obstacle_stop_distance:.1f}m, "
            f"feux respectes={self.respect_traffic_lights}, "
            f"visee anticipee={self.lookahead_distance:.1f}m"
        )

    # ------------------------------------------------------------------
    # Planification d'itineraire
    # ------------------------------------------------------------------

    @staticmethod
    def _wp_key(wp):
        """Cle de deduplication : position arrondie a 2 m, route et voie.

        Le road_id est INDISPENSABLE. Sans lui, les branches d'un
        carrefour -- qui sont autant de routes de liaison distinctes,
        passant par des positions quasi identiques avec le meme lane_id
        -- s'ecrasent mutuellement, et toutes sauf une disparaissent du
        parcours. Mesure avant correction : 62 aretes explorees pour
        62 sauts, soit une seule chaine et aucune ramification, sur une
        ville en damier.
        """
        loc = wp.transform.location
        return (
            round(loc.x / 2.0),
            round(loc.y / 2.0),
            wp.road_id,
            wp.lane_id,
        )

    @staticmethod
    def _rebuild_route(parents, key):
        route = []
        while key is not None:
            wp, parent_key = parents[key]
            route.append(wp)
            key = parent_key
        route.reverse()
        return route

    def _plan_route(self, goal_x, goal_y):
        """Parcours en largeur sur les waypoints CARLA, du vehicule vers le but.

        Retourne (liste de waypoints, longueur en metres), ou (None, inf)
        si aucun itineraire n'existe dans la limite route_max_distance.
        Le pas etant constant, le BFS donne le plus court chemin routier.
        """
        start_loc = self.ego_vehicle.get_location()
        start_wp = self.carla_map.get_waypoint(start_loc, project_to_road=True)
        if start_wp is None:
            return None, float('inf')

        # --- Controle de la projection du point de depart ---
        # get_waypoint() doit rendre un waypoint situe SOUS le
        # vehicule. Quand la carte detenue par ce noeud n'est pas
        # celle de l'episode courant, elle rend un point arbitraire.
        # L'itineraire produit est alors geometriquement valide mais
        # commence ailleurs : le vehicule ne peut pas le suivre,
        # route_index reste bloque, et la visee anticipee finit par
        # retomber sur le but EN LIGNE DROITE, a travers les murs.
        #
        # Ce controle est redondant avec le rafraichissement fait
        # dans _try_find_ego_vehicle. C'est voulu : il transforme un
        # defaut silencieux en defaut visible. Si la situation se
        # represente pour une autre raison, elle apparaitra dans le
        # journal au lieu d'envoyer le vehicule dans un mur.
        ecart = math.hypot(
            start_wp.transform.location.x - start_loc.x,
            start_wp.transform.location.y - start_loc.y,
        )
        if ecart > self.max_projection_error:
            self.get_logger().warn(
                f"PROJECTION ABERRANTE : waypoint de depart a "
                f"{ecart:.1f} m du vehicule "
                f"(seuil {self.max_projection_error:.1f} m). "
                f"Rechargement de la carte et nouvel essai."
            )
            try:
                self.world = self.client.get_world()
                self.carla_map = self.world.get_map()
            except RuntimeError as e:
                self.get_logger().error(
                    f"Rechargement de la carte impossible : {e}"
                )
                return None, float('inf')

            start_wp = self.carla_map.get_waypoint(
                start_loc, project_to_road=True
            )
            if start_wp is None:
                return None, float('inf')

            ecart = math.hypot(
                start_wp.transform.location.x - start_loc.x,
                start_wp.transform.location.y - start_loc.y,
            )
            if ecart > self.max_projection_error:
                self.get_logger().error(
                    f"PROJECTION TOUJOURS ABERRANTE ({ecart:.1f} m) : "
                    f"but rejete. Suivre un itineraire commencant "
                    f"ailleurs enverrait le vehicule hors de la route."
                )
                return None, float('inf')

            self.get_logger().info(
                f"Carte rechargee, projection revenue a {ecart:.1f} m."
            )

        start_key = self._wp_key(start_wp)
        parents = {start_key: (start_wp, None)}
        queue = deque([(start_wp, start_key, 0.0)])

        # Distance du waypoint le plus proche du but effectivement
        # visite. Conservee : c'est elle qui distingue un but manque de
        # peu (tolerance trop juste) d'un but situe hors du reseau
        # explore. C'est cette distinction qui a revele que le
        # generateur travaillait sur une carte perimee -- il annoncait
        # des destinations atteignables que ce parcours ne pouvait pas
        # atteindre, le plus proche waypoint visite restant a 32, 48 puis
        # 59 m des buts proposes.
        meilleur_d = float('inf')

        while queue:
            wp, key, dist = queue.popleft()
            loc = wp.transform.location

            d_but = math.hypot(goal_x - loc.x, goal_y - loc.y)
            if d_but < meilleur_d:
                meilleur_d = d_but

            if d_but <= self.route_goal_tolerance:
                route = self._rebuild_route(parents, key)
                # Longueur GEOMETRIQUE, par opposition a 'dist' qui vaut
                # (nombre de sauts) x route_step sans jamais rien mesurer.
                # C'est la grandeur qui borne la granularite des
                # decisions : un itineraire de 495 m consommait a lui
                # seul 82 % d'un run de 600 m.
                pts = [
                    (w.transform.location.x, w.transform.location.y)
                    for w in route
                ]
                mesuree = sum(
                    math.hypot(pts[i + 1][0] - pts[i][0],
                               pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1)
                )
                self.get_logger().info(
                    f"Itineraire trouve : {len(route)} waypoints, "
                    f"longueur mesuree={mesuree:.1f}m"
                )
                return route, dist

            if dist + self.route_step > self.route_max_distance:
                continue

            for nxt in wp.next(self.route_step):
                nkey = self._wp_key(nxt)
                if nkey in parents:
                    continue
                parents[nkey] = (nxt, key)
                queue.append((nxt, nkey, dist + self.route_step))

        self.get_logger().warn(
            f"Aucun itineraire routier vers "
            f"({goal_x:.2f},{goal_y:.2f}) en moins de "
            f"{self.route_max_distance:.0f}m | "
            f"{len(parents)} waypoints explores, le plus proche a "
            f"{meilleur_d:.1f}m du but "
            f"(tolerance={self.route_goal_tolerance:.1f}m)"
        )
        return None, float('inf')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_goal(self, msg):
        # /active_slam/goal est publie dans le repere 'map' relatif a
        # l'origine SLAM (voir candidate_generator.py). On reconvertit vers
        # le repere CARLA absolu en rajoutant l'origine avant de reinverser Y.
        if self.origin_x is None:
            self.get_logger().warn("Origine CARLA->map pas encore capturee, but ignore.")
            return
        if self.ego_vehicle is None:
            self.get_logger().warn("Vehicule ego pas encore trouve, but ignore.")
            return

        x_carla, y_carla = self._map_to_carla(
            msg.pose.position.x, msg.pose.position.y
        )
        route, route_length = self._plan_route(x_carla, y_carla)

        if route is None:
            # Inatteignable : on le signale tout de suite plutot que de
            # rouler 20 s dans le vide avant que le watchdog ne reagisse.
            self.rejected_goals += 1
            self.get_logger().warn(
                f"BUT REJETE (aucun itineraire routier) : "
                f"x={x_carla:.1f}, y={y_carla:.1f} "
                f"(total rejetes={self.rejected_goals})"
            )
            self.current_goal = None
            self.route = []

            # On publie UNIQUEMENT le rejet, plus /vehicle/arrived.
            #
            # La version precedente publiait aussi 'arrived' pour que le
            # decision_maker redemande une destination. Effet de bord : ce
            # dernier comptait chaque rejet comme une destination atteinte.
            # Sur le run de controle, il annoncait 15 destinations alors que
            # le vehicule n'en avait physiquement atteint que 13, et le
            # budget cessait d'etre identique d'une strategie a l'autre --
            # une strategie qui choisit beaucoup de buts non routables
            # epuisait son budget sans rouler.
            #
            # decision_maker s'abonne desormais a /active_slam/goal_rejected
            # et y reagit en choisissant une nouvelle destination SANS
            # incrementer son compteur. Le comptage devient exact au lieu de
            # reposer sur une heuristique de delai.
            self.goal_rejected_pub.publish(msg)
            return

        self.current_goal = (x_carla, y_carla)
        self.current_goal_msg = msg
        self.route = route
        self.route_index = 0
        self.route_length = route_length
        self.has_arrived_published = False
        self.goal_start_time = time.time()
        self.last_progress_time = time.time()
        self.best_distance_to_goal = float('inf')
        self.obstacle_wait_start = None
        self.last_steer = 0.0
        self.goal_recovery_count = 0

        self.get_logger().info(
            f"Nouvelle destination recue : x={x_carla:.1f}, y={y_carla:.1f} | "
            f"itineraire={len(route)} waypoints, longueur={route_length:.1f}m"
        )

    def on_obstacles(self, msg):
        # Les obstacles arrivent dans le repere 'map' (voir
        # obstacle_detector.py) : on les convertit une fois pour toutes
        # vers le repere CARLA, ou se font tous les calculs de ce noeud.
        if self.origin_x is None:
            return
        self.current_obstacles = [
            self._map_to_carla(p.position.x, p.position.y)
            for p in msg.poses
        ]
    def on_maneuver(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f"Message de manoeuvre invalide (JSON attendu) : {msg.data}")
            return

        action = data.get('action', 'none')
        if action == 'none':
            self.active_maneuver = None
            return

        self.active_maneuver = data
        self.last_maneuver_time = time.time()
        self.get_logger().info(f"Manoeuvre recue : {data}")

    # ------------------------------------------------------------------
    # Perception tactique : obstacles et feux
    # ------------------------------------------------------------------

    def _closest_obstacle_ahead(self, vx, vy, yaw_rad):
        """Distance a l'obstacle le plus proche DANS LE COULOIR de
        circulation du vehicule, ou None si la voie est libre.

        Le critere 'safety' de decision_maker choisit des destinations
        eloignees des obstacles (planification), mais ne dit rien de ce
        qui se trouve sur le chemin : c'est le role de cette verification
        tactique, faite a chaque cycle de controle.

        On projette chaque obstacle dans le repere du vehicule :
          - 'forward' : distance le long de l'axe de marche ;
          - 'lateral' : ecart par rapport a cet axe.
        Un simple filtre angulaire retenait les vehicules en sens
        inverse, sur une voie parallele ou dans une rue perpendiculaire.
        Avec plusieurs dizaines de NPC autour de l'ego, cela le faisait
        freiner en permanence sans qu'aucun obstacle ne soit reellement
        sur sa trajectoire.
        """
        closest = None
        half_width = self.obstacle_corridor_width / 2.0
        cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)

        for ox, oy in self.current_obstacles:
            dx, dy = ox - vx, oy - vy
            forward = dx * cos_yaw + dy * sin_yaw
            lateral = -dx * sin_yaw + dy * cos_yaw

            if forward <= 0.0 or forward > self.obstacle_slow_distance:
                continue
            if abs(lateral) > half_width:
                continue

            dist = math.hypot(dx, dy)
            if closest is None or dist < closest:
                closest = dist

        return closest
    def _map_to_carla(self, x_map, y_map):
        """Repere 'map' de RTAB-Map -> repere CARLA absolu.

        Inverse exact de _carla_to_map() de candidate_generator.py et
        obstacle_detector.py : rotation inverse par le cap de spawn, puis
        inversion de Y, puis translation.
        """
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        dx = x_map * c - y_map * s
        dy = x_map * s + y_map * c
        return dx + self.origin_x, self.origin_y - dy

    def _at_red_light(self):
        """L'ego est-il arrete a un feu rouge ou orange ?

        Les NPC generes par traffic_spawner.py respectent strictement les
        feux (ignore_lights_percentage=0.0). Si l'ego les grillait, il
        percuterait les vehicules arretes devant lui et bloquerait le
        carrefour ; le scenario ne serait alors ni realiste ni
        reproductible. La regle est donc la meme pour lui.
        """
        if not self.respect_traffic_lights:
            return False
        try:
            if not self.ego_vehicle.is_at_traffic_light():
                return False
            state = self.ego_vehicle.get_traffic_light_state()
            return state in (carla.TrafficLightState.Red,
                             carla.TrafficLightState.Yellow)
        except RuntimeError:
            return False

    # ------------------------------------------------------------------
    # Boucle de controle
    # ------------------------------------------------------------------

    def _update_distance(self):
        """Integre la distance parcourue et la publie a 2 Hz.

        L'integration se fait a la cadence de la boucle de controle (10 Hz)
        pour rester precise dans les virages ; seule la publication est
        ralentie, pour ne pas encombrer le graphe ROS.
        """
        try:
            loc = self.ego_vehicle.get_location()
        except RuntimeError:
            return

        position = (loc.x, loc.y)

        if self.last_position is not None:
            pas = math.hypot(
                position[0] - self.last_position[0],
                position[1] - self.last_position[1],
            )
            # Garde-fou : un repositionnement ou un tunnel de teleportation
            # produirait un pas aberrant qui fausserait le budget.
            if pas < 10.0:
                self.distance_traveled += pas

        self.last_position = position

        now = time.time()
        if now - self.last_distance_pub_time >= 0.5:
            self.last_distance_pub_time = now
            self.distance_pub.publish(Float32(data=float(self.distance_traveled)))

    def control_loop(self):
        # --- L'ego detenu est-il toujours vivant ? ---
        # Ce noeud demarre plus vite que bridge_active_slam. Quand un
        # ego d'un run precedent traine encore dans le simulateur, il
        # s'y accroche, puis le pont le detruit en nettoyant les
        # acteurs orphelins. Le noeud pilote alors un acteur mort :
        # get_location() renvoie toujours la meme position, l'origine
        # du repere 'map' est fausse, et le vehicule reel ne bouge
        # jamais.
        #
        # Mesure : origine capturee a (196.08, 5.95) 4,2 s avant que
        # le pont ne spawne le vrai vehicule a (227.3, -1.6).
        # Resultat du run : 0 m parcouru.
        # --- L'origine n'est JAMAIS reprise ---
        # Elle ne dit pas ou est le vehicule : elle DEFINIT le
        # repere 'map', que RTAB-Map fige a son premier scan et que
        # les trois noeuds doivent partager pour tout le run.
        #
        # La version precedente autorisait une recapture pendant les
        # 30 premieres secondes, en supposant le vehicule immobile
        # pendant ce delai. Mesure sur valid_granularite.log :
        # l'origine du detecteur derivait des t+10 s, suivant le
        # vehicule de (227.26,-1.59) a (158.08,-2.62). Les obstacles
        # etaient publies dans un repere glissant, le controleur les
        # reconvertissait avec sa propre origine restee fixe, et ils
        # atterrissaient jusqu'a 70 m a cote : 1 seule detection
        # tactique pour 4 collisions, et un critere de securite
        # calcule sur des obstacles fantomes.
        #
        # La course au demarrage que cette fenetre couvrait est deja
        # traitee par deux gardes permanentes : le rafraichissement
        # de la carte dans _try_find_ego_vehicle, et le controle
        # PROJECTION ABERRANTE dans _plan_route.
        if self.ego_vehicle is not None:
            try:
                vivant = self.ego_vehicle.is_alive
            except RuntimeError:
                vivant = False
            if not vivant:
                self.get_logger().warn(
                    "EGO PERIME : le vehicule suivi a ete detruit. "
                    "Nouvel ego recherche, origine du repere 'map' "
                    "conservee."
                )
                self.ego_vehicle = None
                try:
                    self.world = self.client.get_world()
                    self.carla_map = self.world.get_map()
                except Exception as e:
                    self.get_logger().warn(
                        f"Rafraichissement du monde CARLA "
                        f"impossible : {e}"
                    )
                self.current_goal = None
                self.current_goal_msg = None
                self.route = []
                self.last_steer = 0.0
                # Sans cela, le premier pas d'integration apres le
                # changement d'ego serait le saut entre les deux
                # vehicules.
                self.last_position = None
                return

        if self.ego_vehicle is None:
            self._try_find_ego_vehicle()
            return

        self._update_distance()

        if self.active_maneuver is not None:
            if time.time() - self.last_maneuver_time > self.maneuver_timeout:
                self.active_maneuver = None

        if self.active_maneuver is not None:
            self._apply_maneuver(self.active_maneuver)
            return

        # Desenlisement en cours : il prime sur la poursuite du but.
        # Place apres le traitement des manoeuvres d'urgence, qui restent
        # prioritaires, et avant le test de but pour que le recul se
        # termine meme si le but vient d'etre abandonne.
        if self.recovery_until > 0.0:
            if time.time() < self.recovery_until:
                self._apply_control(
                    throttle=self.recovery_throttle,
                    steer=self.recovery_steer,
                    brake=0.0,
                    reverse=True
                )
                return
            self.recovery_until = 0.0
            self.stall_start = None
            self.last_steer = 0.0
            self.get_logger().info(
                "DESENLISEMENT termine, reprise de la conduite normale."
            )

        if self.current_goal is None:
            self._apply_control(throttle=0.0, steer=0.0, brake=0.3)
            return

        self._pursue_goal()

    def _try_find_ego_vehicle(self):
        self.search_attempts += 1

        # --- Rafraichissement du monde et de la carte ---
        # self.carla_map est capture dans le constructeur, donc
        # AVANT que bridge_active_slam n'ait eventuellement appele
        # load_world(town). get_map() met en cache l'OpenDRIVE de
        # l'episode courant a l'instant de l'appel : au premier run
        # apres un demarrage de CARLA, la carte conservee est celle
        # de l'episode precedent, et elle le reste tout le run.
        #
        # get_actors() interrogeant la simulation en direct, l'ego
        # etait bien trouve et rien ne signalait l'incoherence. Seule
        # la projection trahissait le probleme : ego=(227.26,-1.59)
        # projete sur un waypoint situe a (109.88,-2.39), soit 117 m
        # plus loin. L'itineraire calcule etait correct, mais il
        # commencait a 117 m du vehicule.
        #
        # On rafraichit donc les deux tant que l'ego n'est pas
        # trouve : la derniere lecture est necessairement posterieure
        # au spawn, donc au rechargement de carte.
        try:
            self.world = self.client.get_world()
            self.carla_map = self.world.get_map()
        except RuntimeError as e:
            self.get_logger().warn(
                f"Rafraichissement du monde impossible : {e}"
            )
            return

        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.role_name:
                self.ego_vehicle = actor
                ego_transform = actor.get_transform()
                loc = ego_transform.location
                # --- Capture CONDITIONNELLE de l'origine ---
                # Cette affectation etait inconditionnelle : c'etait une
                # seconde voie de recapture, qui contournait le verrou
                # pose dans la garde EGO PERIME. Il suffisait que ce
                # noeud relache son ego en cours de run pour que le
                # repere 'map' se deplace ici, silencieusement.
                #
                # L'origine ne dit pas ou est le vehicule : elle DEFINIT
                # le repere 'map', que RTAB-Map fige a son premier scan.
                if self.origin_x is None:
                    self.origin_x = loc.x
                    self.origin_y = loc.y
                    self.origin_yaw = math.radians(
                        -ego_transform.rotation.yaw
                    )
                    self.get_logger().info(
                        f"Vehicule ego trouve, origine CARLA->map "
                        f"capturee : ({self.origin_x:.2f}, "
                        f"{self.origin_y:.2f})"
                    )
                else:
                    self.get_logger().info(
                        f"Nouvel ego trouve, origine du repere 'map' "
                        f"conservee : ({self.origin_x:.2f}, "
                        f"{self.origin_y:.2f})"
                    )
                # Trace permanente : permet de verifier d'un grep que les
                # trois noeuds partagent bien la meme carte.
                self.get_logger().info(
                    f"Carte active : {self.carla_map.name}"
                )
                self.get_logger().info("Vehicule ego trouve, demarrage du controle")
                return

        if self.search_attempts % 20 == 0:
            self.get_logger().warn("Vehicule ego introuvable, attente du spawn CARLA...")

    def _pursue_goal(self):
        transform = self.ego_vehicle.get_transform()
        vx, vy = transform.location.x, transform.location.y
        yaw_rad = math.radians(transform.rotation.yaw)
        gx, gy = self.current_goal

        final_distance = math.hypot(gx - vx, gy - vy)

        # --- Arrivee ---
        if final_distance <= self.arrival_tolerance:
            self._apply_control(throttle=0.0, steer=0.0, brake=0.6)
            if not self.has_arrived_published:
                self.reached_goals += 1
                self.arrived_pub.publish(Bool(data=True))
                self.has_arrived_published = True
                self.get_logger().info(
                    f"Destination atteinte (total atteints={self.reached_goals}) | "
                    f"distance parcourue={self.distance_traveled:.1f}m"
                )
            self.current_goal = None
            self.route = []
            self.last_steer = 0.0
            return

        now = time.time()

        # --- Surveillance de progression ---
        # Filet de securite : meme avec un itineraire valide, le vehicule
        # peut rester bloque (obstacle, sortie de route, collision).
        if final_distance < self.best_distance_to_goal - self.min_progress:
            self.best_distance_to_goal = final_distance
            self.last_progress_time = now

        elapsed = now - self.goal_start_time
        stalled = now - self.last_progress_time

        if elapsed > self.goal_timeout or stalled > self.no_progress_timeout:
            self.abandoned_goals += 1
            self.get_logger().warn(
                f"BUT ABANDONNE (pas de progression) : "
                f"distance={final_distance:.1f}m, "
                f"meilleure={self.best_distance_to_goal:.1f}m, "
                f"ecoule={elapsed:.1f}s, sans_progres={stalled:.1f}s "
                f"(total abandonnes={self.abandoned_goals})"
            )
            self._apply_control(throttle=0.0, steer=0.0, brake=0.6)
            self.current_goal = None
            self.route = []
            self.last_steer = 0.0

            # --- Un abandon est un REJET, pas une arrivee ---
            # Cette branche publiait /vehicle/arrived. Le filtre de
            # decision_maker.on_arrived ne rejette que les
            # republications survenant moins d'une seconde apres la
            # publication du but ; un abandon arrive apres ~70 s. Il
            # passait donc le filtre et incrementait
            # destinations_reached.
            #
            # Consequence sur la campagne comparative : une strategie
            # qui s'enlise souvent affichait PLUS de destinations
            # atteintes qu'une strategie qui roule. La metrique
            # s'inversait.
            #
            # On publie donc sur /active_slam/goal_rejected, comme
            # pour un but non routable : le point entre dans
            # unreachable_points, une nouvelle destination est
            # demandee, et le compteur n'est pas incremente.
            if self.current_goal_msg is not None:
                self.goal_rejected_pub.publish(self.current_goal_msg)
            elif not self.has_arrived_published:
                # Repli : sans message d'origine, on retombe sur
                # l'ancien comportement plutot que de laisser le
                # vehicule sans destination.
                self.arrived_pub.publish(Bool(data=True))
                self.has_arrived_published = True
            return

        if not self.route:
            self._apply_control(throttle=0.0, steer=0.0, brake=0.5)
            return

        # --- Respect des feux tricolores ---
        # Les NPC s'arretent au rouge : l'ego doit faire de meme, sinon il
        # les percute et bloque le carrefour. Attendre au feu n'est pas un
        # blocage, le chien de garde de progression est donc suspendu.
        if self._at_red_light():
            self._apply_control(throttle=0.0, steer=0.0, brake=1.0)
            self.last_steer = 0.0
            self.last_progress_time = now
            if now - self.last_light_log_time > 3.0:
                self.last_light_log_time = now
                self.get_logger().info("FEU ROUGE : arret et attente.")
            return

        # --- Distance de securite avec les obstacles devant ---
        obstacle_distance = self._closest_obstacle_ahead(vx, vy, yaw_rad)

        if obstacle_distance is not None and obstacle_distance <= self.obstacle_stop_distance:
            # Trop pres : arret complet, on attend que la voie se degage.
            self._apply_control(throttle=0.0, steer=0.0, brake=1.0)

            if self.obstacle_wait_start is None:
                self.obstacle_wait_start = now
            waited = now - self.obstacle_wait_start

            # Attendre derriere un vehicule arrete a un feu rouge est un
            # comportement normal, pas un blocage : on suspend le chien de
            # garde de progression pendant l'attente, sinon le but serait
            # abandonne au bout de no_progress_timeout alors que la voie
            # va se degager. Au-dela de max_obstacle_wait, l'obstacle est
            # considere comme definitif (vehicule gare, epave) et le chien
            # de garde reprend la main.
            if waited < self.max_obstacle_wait:
                self.last_progress_time = now

            if now - self.last_obstacle_log_time > 2.0:
                self.last_obstacle_log_time = now
                self.get_logger().warn(
                    f"OBSTACLE DEVANT a {obstacle_distance:.1f}m "
                    f"(seuil={self.obstacle_stop_distance:.1f}m) : "
                    f"attente {waited:.0f}s/{self.max_obstacle_wait:.0f}s"
                )
            return

        # Voie degagee : on remet le compteur d'attente a zero.
        self.obstacle_wait_start = None

        # --- Avancement le long de l'itineraire planifie ---
        while self.route_index < len(self.route) - 1:
            wp_loc = self.route[self.route_index].transform.location
            if math.hypot(wp_loc.x - vx, wp_loc.y - vy) <= self.waypoint_reach_tolerance:
                self.route_index += 1
                # Progression reelle le long de l'itineraire : on reinitialise
                # le chien de garde meme si la distance a vol d'oiseau vers le
                # but final n'a pas diminue (un itineraire routier peut
                # legitimement s'eloigner temporairement du but en ligne
                # droite, par exemple pour contourner un pate de maisons).
                self.last_progress_time = now
            else:
                break

        # --- Visee anticipee (pure pursuit) ---
        # On ne vise pas le waypoint courant, souvent a 2-3 m, mais le
        # premier point de l'itineraire situe au moins a
        # lookahead_distance. Viser trop pres fait sauter la cible a chaque
        # waypoint valide, d'ou les embardees. Si aucun point n'est assez
        # loin, on est sur le dernier troncon et on vise directement le but.
        target_x, target_y = gx, gy
        for i in range(self.route_index, len(self.route)):
            loc = self.route[i].transform.location
            if math.hypot(loc.x - vx, loc.y - vy) >= self.lookahead_distance:
                target_x, target_y = loc.x, loc.y
                break

        dx, dy = target_x - vx, target_y - vy
        target_heading = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_heading - yaw_rad)

        # --- Itineraire devenu insuivable ---
        # waypoint.next() ne parcourt le graphe routier que vers
        # l'AVANT : l'itineraire a ete calcule pour le cap qu'avait le
        # vehicule quand le but a ete recu. S'il s'est retrouve a
        # contresens, ce chemin commence derriere lui et il ne peut plus
        # le rejoindre.
        #
        # Trace mesuree sur six cycles consecutifs (valid_trajet.log) :
        #   wp=0/5 cap_err=-138deg steer=-0.15
        #   wp=0/5 cap_err=-123deg steer=-1.00
        #   wp=0/5 cap_err= -87deg steer=-1.00
        #   wp=0/5 cap_err=+156deg steer=+1.00
        # Le vehicule tourne sur lui-meme, la distance au but reste a
        # 32 m. Ni le chien de garde de progression (la distance varie
        # de quelques centimetres) ni le desenlisement (le vehicule
        # bouge, 0,5 a 1,2 m/s) ne reagissent.
        #
        # Les deux conditions sont necessaires : route_index > 0 signifie
        # que le chemin est reellement suivi, et une erreur de cap
        # passagere au-dela de 100 deg est normale dans un virage serre.
        if self.route_index == 0 and abs(heading_error) > math.radians(100):
            if self.wrong_way_start is None:
                self.wrong_way_start = now
            elif now - self.wrong_way_start > self.wrong_way_timeout:
                self.abandoned_goals += 1
                self.get_logger().warn(
                    f"ITINERAIRE INSUIVABLE : premier waypoint jamais "
                    f"atteint et cap oppose "
                    f"({math.degrees(heading_error):.0f}deg) depuis "
                    f"{now - self.wrong_way_start:.0f}s. Le vehicule est "
                    f"probablement a contresens : but rejete et "
                    f"replanification depuis le cap actuel "
                    f"(total abandonnes={self.abandoned_goals})."
                )
                self._apply_control(throttle=0.0, steer=0.0, brake=0.6)
                self.current_goal = None
                self.route = []
                self.last_steer = 0.0
                self.wrong_way_start = None
                if self.current_goal_msg is not None:
                    self.goal_rejected_pub.publish(self.current_goal_msg)
                return
        else:
            self.wrong_way_start = None

        # --- Braquage adouci et limite en vitesse ---
        # Gain sur steer_gain_deg au lieu des 45 deg d'origine, puis
        # limitation de la variation par cycle : le volant ne peut plus
        # passer d'une butee a l'autre en un dixieme de seconde.
        raw_steer = max(-1.0, min(1.0, heading_error / self.steer_gain_rad))
        delta = max(-self.max_steer_rate,
                    min(self.max_steer_rate, raw_steer - self.last_steer))
        steer = self.last_steer + delta
        self.last_steer = steer

        velocity = self.ego_vehicle.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)

        if now - self.last_pursue_log_time > 2.0:
            self.last_pursue_log_time = now
            self.get_logger().info(
                f"POURSUITE : dist_but={final_distance:.1f}m "
                f"wp={self.route_index}/{len(self.route) - 1} "
                f"cap_err={math.degrees(heading_error):.0f}deg "
                f"vitesse={speed:.2f}m/s steer={steer:.2f}"
            )

        # --- Detection d'enlisement ---
        # A ce point de la fonction, tous les arrets legitimes ont deja
        # provoque un return : arrivee, but abandonne, itineraire vide,
        # feu rouge, obstacle a l'arret devant. Si le vehicule est encore
        # ici et ne bouge pas, c'est qu'il subit un blocage PHYSIQUE -
        # typiquement un mur ou un trottoir, invisible pour
        # obstacle_detector qui n'enumere que vehicules et pietons.
        if speed < self.recovery_speed_threshold:
            if self.stall_start is None:
                self.stall_start = now
            elif now - self.stall_start > self.recovery_stall_time:
                self.recovery_count += 1
                self.goal_recovery_count += 1

                # --- Braquage ALTERNE ---
                # La version precedente prenait l'oppose du dernier
                # braquage : recovery_steer = -last_steer. Or quand
                # le vehicule pousse droit dans un mur, last_steer
                # vaut ~0 : il reculait tout droit, repartait tout
                # droit, et retrouvait le meme obstacle. Trace du run
                # de chauffe, distance au but apres chaque recul :
                # 28.2 -> 25.0, 28.1 -> 25.0, 28.5 -> 25.0. Trois fois
                # la meme valeur : un cycle deterministe, pas des
                # tentatives.
                #
                # Alterner pleine butee d'un cote puis de l'autre est
                # la seule facon de garantir que chaque tentative part
                # d'une geometrie differente de la precedente.
                self.recovery_steer = (
                    0.9 if (self.recovery_count % 2 == 1) else -0.9
                )
                self.recovery_until = now + self.recovery_duration
                self.stall_start = None
                self.get_logger().warn(
                    f"ENLISEMENT DETECTE : vitesse={speed:.2f}m/s pendant "
                    f"{self.recovery_stall_time:.0f}s | recul de "
                    f"{self.recovery_duration:.1f}s, braquage="
                    f"{self.recovery_steer:+.1f} "
                    f"(tentative {self.goal_recovery_count}/"
                    f"{self.max_recoveries_per_goal} sur ce but, "
                    f"total desenlisements={self.recovery_count})"
                )
                self._apply_control(
                    throttle=self.recovery_throttle,
                    steer=self.recovery_steer,
                    brake=0.0,
                    reverse=True
                )

                # --- Abandon anticipe ---
                # Sans ce test il faut attendre no_progress_timeout
                # (45 s), soit 4 a 5 enlisements au meme endroit, pour
                # renoncer. Le recul en cours se termine quand meme :
                # control_loop traite le desenlisement AVANT le test
                # de but.
                if self.goal_recovery_count >= self.max_recoveries_per_goal:
                    self.abandoned_goals += 1
                    self.get_logger().warn(
                        f"BUT ABANDONNE (enlisements repetes) : "
                        f"{self.goal_recovery_count} desenlisements "
                        f"sans progres "
                        f"(total abandonnes={self.abandoned_goals})"
                    )
                    self.current_goal = None
                    self.route = []
                    self.last_steer = 0.0
                    if self.current_goal_msg is not None:
                        self.goal_rejected_pub.publish(
                            self.current_goal_msg
                        )
                return
        else:
            self.stall_start = None

        # Cible derriere le vehicule : on braque a fond et on avance au
        # ralenti pour se reorienter, plutot que de freiner indefiniment
        # (ce qui figeait le vehicule sans issue possible).
        if abs(heading_error) > math.radians(100):
            full_lock = 1.0 if heading_error > 0 else -1.0
            self.last_steer = full_lock
            self._apply_control(
                throttle=self.min_throttle,
                steer=full_lock,
                brake=0.0
            )
            return

        # Reduction progressive du throttle en fonction de l'erreur de cap :
        # le vehicule ralentit a l'approche d'un virage serre, ce qui laisse
        # a l'ICP le temps de suivre d'un scan a l'autre.
        heading_factor = 1.0 - min(abs(heading_error) / math.radians(100), 1.0)
        throttle = self.max_throttle * heading_factor
        throttle = min(throttle, self.max_throttle * (1.0 - min(abs(steer), 0.8)))

        # Plancher de throttle : en dessous d'environ 0.3, un vehicule a
        # l'arret ne demarre pas dans CARLA. Sans ce plancher, un fort ecart
        # de cap donne un throttle si faible que le vehicule reste immobile,
        # l'ecart de cap ne change donc jamais, et le blocage est definitif.
        if speed < 0.5:
            throttle = max(throttle, self.min_throttle)

        # Ralentissement progressif a l'approche d'un obstacle : entre
        # obstacle_slow_distance et obstacle_stop_distance, le throttle
        # decroit lineairement, ce qui evite d'arriver lance sur la zone
        # d'arret et de la depasser par inertie. Ce plafond est applique
        # APRES le plancher min_throttle : approcher un obstacle doit
        # pouvoir ramener le throttle en dessous du plancher de demarrage.
        if obstacle_distance is not None:
            ratio = (
                (obstacle_distance - self.obstacle_stop_distance)
                / max(self.obstacle_slow_distance - self.obstacle_stop_distance, 0.1)
            )
            throttle = min(throttle, self.max_throttle * max(0.0, min(1.0, ratio)))
            if now - self.last_obstacle_log_time > 2.0:
                self.last_obstacle_log_time = now
                self.get_logger().info(
                    f"OBSTACLE PROCHE a {obstacle_distance:.1f}m : "
                    f"ralentissement (throttle={throttle:.2f})"
                )

        # Freinage en virage : lever le pied ne freine pas. Vitesse max
        # admissible 9 m/s en ligne droite, 3 m/s a fond de braquage.
        vmax = 9.0 - 6.0 * min(abs(steer), 1.0)
        brake = min(1.0, max(0.0, (speed - vmax) / 3.0))
        if brake > 0.0:
            throttle = 0.0
        self._apply_control(throttle=throttle, steer=steer, brake=brake)

    def _apply_maneuver(self, maneuver):
        action = maneuver.get('action')
        if action == 'stop':
            self._apply_control(throttle=0.0, steer=0.0, brake=1.0)
        elif action == 'reverse':
            self._apply_control(throttle=0.3, steer=0.0, brake=0.0, reverse=True)
        elif action == 'swerve':
            direction = maneuver.get('direction', 'left')
            steer = -0.6 if direction == 'left' else 0.6
            self.last_steer = steer
            self._apply_control(throttle=0.25, steer=steer, brake=0.0)
        else:
            self.get_logger().warn(f"Action de manoeuvre inconnue : {action}")
            self._apply_control(throttle=0.0, steer=0.0, brake=1.0)

    def _apply_control(self, throttle, steer, brake, reverse=False):
        control = carla.VehicleControl()
        control.throttle = max(0.0, min(1.0, throttle))
        control.steer = max(-1.0, min(1.0, steer))
        control.brake = max(0.0, min(1.0, brake))
        control.reverse = reverse
        self.ego_vehicle.apply_control(control)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerActiveSLAM()
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
