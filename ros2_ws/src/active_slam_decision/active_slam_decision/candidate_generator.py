#!/usr/bin/env python3
"""
Genere des destinations candidates ATTEIGNABLES sur le reseau routier
CARLA, autour de la position courante du vehicule ego, et les publie sur
/active_slam/candidate_goals (geometry_msgs/msg/PoseArray) pour le noeud
active_slam_decision_maker.

Pourquoi un parcours du graphe et non un filtre par rayon
---------------------------------------------------------
La version precedente prenait TOUS les waypoints de la carte
(generate_waypoints) et ne gardait que ceux situes dans un rayon
euclidien autour du vehicule. Ce filtre ignore completement le sens de
circulation : il proposait des destinations situees derriere le
vehicule, ou de l'autre cote de rues a sens unique. Le controleur
decouvrait ensuite, une par une, qu'elles etaient inaccessibles et les
rejetait. Avec 373 candidats espaces de 8 m dans une zone devenue
inatteignable, le systeme entrait en boucle : choisir -> rejeter ->
choisir, plusieurs fois par seconde, sans que le vehicule ne bouge (le
controleur freine tant qu'il n'a pas de but valide).

Ce noeud parcourt donc le graphe routier vers l'AVANT depuis la
position de l'ego, via waypoint.next(), exactement comme le fait
_plan_route() dans vehicle_controller_active_slam.py. Trois
consequences :

  - tout candidat publie est atteignable en respectant le code de la
    route : le taux de rejet tombe a zero et la boucle disparait ;
  - la distance associee a chaque candidat est une VRAIE distance
    routiere, et non une distance a vol d'oiseau. C'est la grandeur
    demandee par le sujet pour le critere de cout de deplacement ;
  - le generateur et le controleur partagent la meme definition de
    l'atteignabilite, donc ils ne peuvent plus se contredire.

Filtres appliques aux candidats
-------------------------------
  - lane_type == Driving : exclut trottoirs, parkings, voies de service ;
  - not is_junction : une destination au milieu d'un carrefour ferait
    stationner le vehicule dans l'intersection. Les carrefours restent
    traverses pendant le parcours, ils ne sont simplement jamais retenus
    comme destination ;
  - distance routiere >= min_candidate_distance : en dessous, le
    decideur choisit systematiquement le point le plus proche (critere
    de distance) et l'exploration degenere en sauts de quelques metres.

Conversion de repere
--------------------
CARLA utilise un repere main gauche, ROS/RTAB-Map un repere main droite.
On soustrait l'origine SLAM puis on inverse Y :
    x_map = x_carla - origin_x
    y_map = -(y_carla - origin_y)
vehicle_controller_active_slam.py applique exactement l'inverse. Les deux
noeuds capturent l'origine sur la position de spawn de l'ego, avant tout
deplacement, donc les deux origines coincident.

Le filtrage ne verifie pas la presence d'obstacles ou de trafic : c'est
le role du critere 'safety' de decision_maker (safety_evaluator.py) au
niveau strategique, et de la verification tactique du controleur au
niveau de la conduite.
"""

import math
import os
from collections import deque

import carla

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped


class CandidateGenerator(Node):

    def __init__(self):
        super().__init__('active_slam_candidate_generator')

        self.declare_parameter('carla_host', '127.0.0.1')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('role_name', 'ego')

        # Pas du parcours du graphe routier. Doit rester coherent avec
        # route_step du controleur pour que les deux voient le meme reseau.
        self.declare_parameter('waypoint_spacing', 8.0)

        # Portee du parcours, en distance ROUTIERE et non a vol d'oiseau.
        self.declare_parameter('max_route_distance', 250.0)

        # Distance routiere minimale d'une destination. Relevee par rapport
        # aux 5 m d'origine : en dessous, l'exploration degenere en sauts de
        # quelques metres et aucune strategie ne peut se distinguer d'une
        # autre lors de la campagne comparative.
        self.declare_parameter('min_candidate_distance', 25.0)

        # Plafond de securite a vol d'oiseau (0 = desactive). Evite de
        # proposer un point tres eloigne geographiquement que le reseau
        # routier rendrait accessible par un long detour.
        self.declare_parameter('search_radius', 0.0)

        # Garde-fou sur la taille du message publie.
        self.declare_parameter('max_candidates', 400)

        # --- ZONE BORNEE ---
        # Demi-cote du carre centre sur le point d'apparition, dans
        # le repere 'map'. Passe par l'environnement plutot que par
        # le fichier de lancement : un seul fichier du workspace est
        # ainsi modifie, comme pour la resolution camera (variante F).
        # 0 = desactive : comportement strictement inchange.
        self.declare_parameter(
            'zone_half_size',
            float(os.environ.get('ZONE_HALF_SIZE', 0.0)))

        self.declare_parameter('publish_period', 3.0)

        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        self.role_name = self.get_parameter('role_name').value
        self.waypoint_spacing = float(self.get_parameter('waypoint_spacing').value)
        self.max_route_distance = float(self.get_parameter('max_route_distance').value)
        self.min_candidate_distance = float(self.get_parameter('min_candidate_distance').value)
        self.search_radius = float(self.get_parameter('search_radius').value)
        self.max_candidates = int(self.get_parameter('max_candidates').value)
        self.zone_half_size = float(
            self.get_parameter('zone_half_size').value)
        publish_period = float(self.get_parameter('publish_period').value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization_pose',
            self.on_pose, 10
        )
        self.pub = self.create_publisher(PoseArray, '/active_slam/candidate_goals', 10)

        self.get_logger().info(f"Connexion a CARLA {host}:{port} ...")
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

        self.ego_vehicle = None

        self.get_logger().info(
            f"Generation par parcours du reseau routier : pas={self.waypoint_spacing:.1f}m, "
            f"portee={self.max_route_distance:.0f}m de route, "
            f"distance min={self.min_candidate_distance:.0f}m"
        )

        if self.zone_half_size > 0.0:
            self.get_logger().info(
                f"ZONE BORNEE : carre de "
                f"{2 * self.zone_half_size:.0f} m de cote, centre sur "
                f"le point d'apparition (repere map).")

        self.timer = self.create_timer(publish_period, self.publish_candidates)

    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------

    def on_pose(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    def _carla_to_map(self, x, y):
        """Repere CARLA absolu -> repere 'map' de RTAB-Map.

        RTAB-Map construit son repere aligne sur le CAP INITIAL du vehicule.
        La conversion demande donc trois operations et non deux : translation
        par le spawn, inversion de Y (main gauche -> main droite), puis
        ROTATION par le cap de spawn. Sans cette rotation, les candidats
        publies ici et la pose SLAM du decideur vivent dans deux reperes
        tournes l'un par rapport a l'autre des que le vehicule ne demarre pas
        cap vers CARLA +x.
        """
        dx = x - self.origin_x
        dy = -(y - self.origin_y)
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return dx * c + dy * s, -dx * s + dy * c
    def _find_ego_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.role_name:
                return actor
        return None

    @staticmethod
    def _wp_key(wp):
        """Cle de deduplication : position arrondie a 2 m, route et voie.

        DOIT rester identique a celle du controleur : c'est ce qui
        garantit que les deux noeuds voient le meme reseau et donnent
        la meme reponse a la question 'puis-je aller la ?'.

        Le road_id est indispensable. Sans lui, les branches d'un
        carrefour s'ecrasent mutuellement. Ce noeud y echappait
        partiellement parce qu'il parcourt avec un pas de 8 m, qui
        enjambe l'interieur des carrefours -- d'ou la contradiction
        observee : 20 destinations annoncees atteignables ici, aucune
        routable par le controleur, qui avance lui de 4 m.
        """
        loc = wp.transform.location
        return (
            round(loc.x / 2.0),
            round(loc.y / 2.0),
            wp.road_id,
            wp.lane_id,
        )

    # ------------------------------------------------------------------
    # Parcours du graphe routier
    # ------------------------------------------------------------------

    def _reachable_candidates(self, vehicle_location):
        """Parcours en largeur vers l'avant depuis la position de l'ego.

        Retourne une liste de (waypoint, distance_routiere) triee par
        distance croissante. waypoint.next() ne progresse que dans le sens
        de circulation : tout ce qui sort de ce parcours est donc
        atteignable par le vehicule sans infraction.
        """
        start_wp = self.carla_map.get_waypoint(vehicle_location, project_to_road=True)
        if start_wp is None:
            return []

        start_key = self._wp_key(start_wp)
        visited = {start_key}
        queue = deque([(start_wp, 0.0)])
        candidates = []

        while queue:
            wp, dist = queue.popleft()

            if dist >= self.min_candidate_distance:
                loc = wp.transform.location
                keep = True
                # Une destination au milieu d'un carrefour ferait
                # stationner le vehicule dans l'intersection.
                if wp.is_junction:
                    keep = False
                elif wp.lane_type != carla.LaneType.Driving:
                    keep = False
                elif self.search_radius > 0.0:
                    straight = math.hypot(loc.x - vehicle_location.x,
                                          loc.y - vehicle_location.y)
                    if straight > self.search_radius:
                        keep = False
                if keep:
                    candidates.append((wp, dist))
                    if len(candidates) >= self.max_candidates:
                        break

            if dist + self.waypoint_spacing > self.max_route_distance:
                continue

            for nxt in wp.next(self.waypoint_spacing):
                nkey = self._wp_key(nxt)
                if nkey in visited:
                    continue
                visited.add(nkey)
                queue.append((nxt, dist + self.waypoint_spacing))

        return self._filtrer_zone(candidates)

    def _filtrer_zone(self, candidates):
        """Ne garde que les candidats situes dans la zone bornee.

        La zone materialise la tache : cartographier completement une
        region donnee, et non parcourir une distance. C'est la seule
        asymetrie introduite par rapport a la reference -- et elle est
        honnete, une trajectoire predefinie ne pouvant par construction
        tenir compte d'une carte en cours de construction.

        REPLI INDISPENSABLE. Si le vehicule est sorti de la zone, aucun
        candidat interieur n'est necessairement atteignable dans la
        portee de parcours. Retourner une liste vide priverait le
        decideur de tout but et immobiliserait le vehicule : le run
        serait perdu sans qu'aucune erreur ne soit levee. On retourne
        alors les candidats les plus PROCHES DU CENTRE, ce qui ramene
        le vehicule vers la zone.
        """
        if self.zone_half_size <= 0.0 or self.origin_x is None:
            return candidates

        dedans, dehors = [], []
        for wp, dist in candidates:
            loc = wp.transform.location
            mx, my = self._carla_to_map(loc.x, loc.y)
            if (abs(mx) <= self.zone_half_size
                    and abs(my) <= self.zone_half_size):
                dedans.append((wp, dist))
            else:
                dehors.append((wp, dist, math.hypot(mx, my)))

        if dedans:
            return dedans
        if not dehors:
            return []

        dehors.sort(key=lambda t: t[2])
        garde = dehors[:8]
        self.get_logger().warn(
            f"HORS ZONE : aucun candidat interieur atteignable, "
            f"repli sur les {len(garde)} plus proches du centre "
            f"(le plus proche a {garde[0][2]:.0f} m).")
        return [(wp, dist) for wp, dist, _ in garde]

    # ------------------------------------------------------------------
    # Publication
    # ------------------------------------------------------------------

    def publish_candidates(self):
        # --- L'ego detenu est-il toujours vivant ? ---
        # Meme garde que dans vehicle_controller_active_slam : un ego
        # d'un run precedent peut etre trouve avant que le pont ne
        # l'ait detruit et remplace. L'origine capturee serait alors
        # celle du mauvais vehicule, et TOUS les candidats publies
        # vivraient dans un repere decale.
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
                    "EGO PERIME : vehicule suivi detruit, nouvel ego "
                    "recherche. Origine du repere 'map' conservee."
                )
                self.ego_vehicle = None
                # Rafraichir le monde AVANT de rechercher : un
                # acteur tire d'un episode perime rapporte
                # is_alive == False alors que le vehicule est bien
                # vivant. C'est la correction deja en service cote
                # controleur (_try_find_ego_vehicle).
                try:
                    self.world = self.client.get_world()
                except Exception as e:
                    self.get_logger().warn(
                        f"Rafraichissement du monde CARLA "
                        f"impossible : {e}"
                    )
                return

        if self.ego_vehicle is None:
            # --- Rafraichir le monde ET la carte avant de chercher ---
            # self.carla_map est capture dans le constructeur, donc
            # AVANT que bridge_active_slam n'ait recharge la ville.
            # get_map() met en cache l'OpenDRIVE de l'episode courant a
            # l'instant de l'appel : la carte conservee est alors celle
            # de l'episode PRECEDENT, et elle le reste tout le run.
            #
            # Mesure sur valid_bfs.log : ce noeud annoncait 23
            # destinations atteignables en 32-248 m de route pendant que
            # le controleur -- qui rafraichit deja sa carte dans
            # _try_find_ego_vehicle -- n'explorait qu'une chaine de 53
            # waypoints le long d'une seule rue, a y ~ -3.5, et ne
            # trouvait aucun itineraire. Les deux noeuds parcouraient
            # deux reseaux routiers differents.
            #
            # La derniere lecture faite ici est necessairement
            # posterieure au spawn de l'ego, donc au rechargement de la
            # ville.
            try:
                self.world = self.client.get_world()
                self.carla_map = self.world.get_map()
            except Exception as e:
                self.get_logger().warn(
                    f"Rafraichissement du monde CARLA impossible : {e}"
                )
            self.ego_vehicle = self._find_ego_vehicle()
            if self.ego_vehicle is None:
                self.get_logger().warn("Vehicule ego introuvable dans CARLA, en attente...")
                return
            # Trace permanente : permet de verifier d'un grep que les
            # trois noeuds partagent bien la meme carte.
            self.get_logger().info(
                f"Carte active : {self.carla_map.name}"
            )

        ego_transform = self.ego_vehicle.get_transform()
        vehicle_location = ego_transform.location

        if self.origin_x is None:
            # Capture, une seule fois, la POSE de spawn (position ET cap) :
            # c'est elle qui definit l'origine du repere 'map' de RTAB-Map.
            # A ce stade le vehicule n'a pas encore recu de destination, donc
            # il n'a pas bouge : cela coincide avec le debut de la
            # trajectoire SLAM.
            self.origin_x = vehicle_location.x
            self.origin_y = vehicle_location.y
            self.origin_yaw = math.radians(-ego_transform.rotation.yaw)
            self.get_logger().info(
                f"Origine CARLA->map capturee : ({self.origin_x:.2f}, "
                f"{self.origin_y:.2f}, cap={math.degrees(self.origin_yaw):.1f}deg)"
            )

        candidates = self._reachable_candidates(vehicle_location)

        if not candidates:
            self.get_logger().warn(
                "Aucune destination atteignable depuis la position courante "
                f"(portee={self.max_route_distance:.0f}m de route). Le vehicule "
                "est peut-etre dans une impasse ou hors chaussee."
            )
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for wp, dist_routiere in candidates:
            loc = wp.transform.location
            pose = Pose()
            pose.position.x, pose.position.y = self._carla_to_map(loc.x, loc.y)
            # --- z transporte la DISTANCE ROUTIERE ---
            # Ce champ etait inutilise (tous les candidats sont au
            # niveau du sol). Il porte desormais la distance reellement
            # a parcourir, calculee par le parcours du graphe ci-dessus.
            #
            # Sans elle, decision_maker penalisait la distance a vol
            # d'oiseau : mesure sur valid_capteur40.log, un candidat a
            # 80 m a vol d'oiseau demandait 495 m de route, soit un
            # facteur 6,2. Le critere de cout de deplacement classait
            # les candidats sur une grandeur sans rapport avec ce que le
            # vehicule allait reellement parcourir.
            pose.position.z = float(dist_routiere)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)

        distances = [d for _wp, d in candidates]
        self.get_logger().info(
            f"{len(candidates)} destinations candidates ATTEIGNABLES publiees "
            f"(distance routiere {min(distances):.0f}-{max(distances):.0f}m)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CandidateGenerator()
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
