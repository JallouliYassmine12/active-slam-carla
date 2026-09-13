#!/usr/bin/env python3
"""
Detecte les obstacles (vehicules et pietons) autour du vehicule ego dans
CARLA, et les publie sur /obstacle_detection/obstacles
(geometry_msgs/msg/PoseArray) pour le noeud active_slam_decision_maker
(voir safety_evaluator.py / on_obstacles dans decision_maker.py).

Se connecte directement au serveur CARLA (comme candidate_generator.py) et
interroge la position des acteurs (autres vehicules, pietons) proches du
vehicule ego, plutot que de traiter un nuage de points LiDAR : c'est le
choix le plus simple et le plus fiable en simulation (position exacte
donnee par le simulateur), et il reste dans le meme esprit que
candidate_generator.py qui interroge deja directement l'API CARLA.

Convention de repere (IMPORTANTE)
---------------------------------
Le repere 'map' est celui de RTAB-Map. RTAB-Map le construit aligne sur le
CAP INITIAL du vehicule : son axe x pointe vers l'avant au moment du spawn.
La conversion depuis CARLA demande donc TROIS operations, et pas deux :

  1. translation par la position de spawn ;
  2. inversion de Y (CARLA est en main gauche, ROS en main droite) ;
  3. ROTATION par le cap de spawn.

L'etape 3 etait absente des versions precedentes. Tant que le vehicule
demarrait cap vers CARLA +x, l'erreur restait invisible. Des que le spawn
regarde ailleurs (Town03_Opt, point 0 : cap vers -x), les positions
publiees ici et la pose SLAM du decideur se retrouvaient dans deux reperes
tournes l'un par rapport a l'autre. Les criteres de gain d'information,
d'incertitude et de fermeture de boucle etaient alors calcules sur des
coordonnees incoherentes.

candidate_generator.py et vehicle_controller_active_slam.py appliquent
exactement la meme convention (et son inverse cote controleur).
"""
import math
import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('active_slam_obstacle_detector')

        self.declare_parameter('carla_host', '127.0.0.1')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('role_name', 'ego')
        self.declare_parameter('detection_radius', 100.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('obstacles_topic', '/obstacle_detection/obstacles')

        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        self.role_name = self.get_parameter('role_name').value
        self.detection_radius = self.get_parameter('detection_radius').value
        publish_period = self.get_parameter('publish_period').value
        obstacles_topic = self.get_parameter('obstacles_topic').value

        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None
        self.ego_vehicle = None

        self.get_logger().info(f"Connexion a CARLA {host}:{port} ...")
        # --- Le client est CONSERVE ---
        # Sans lui, self.world reste fige sur l'episode CARLA en
        # cours a la construction du noeud, donc AVANT le
        # rechargement de carte fait par bridge_active_slam. Les
        # acteurs tires de cet episode perime rapportent
        # is_alive == False alors que le vehicule est bien vivant :
        # 61 declenchements de la garde EGO PERIME mesures sur un
        # run de 118 s.
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.obstacles_pub = self.create_publisher(
            PoseArray,
            obstacles_topic,
            10
        )

        self.create_timer(publish_period, self.publish_obstacles)

    def _find_ego_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.role_name:
                return actor
        return None

    def _carla_to_map(self, x, y):
        """Repere CARLA absolu -> repere 'map' de RTAB-Map.

        Translation par le spawn, inversion de Y (main gauche -> main
        droite), puis rotation par le cap de spawn.
        """
        dx = x - self.origin_x
        dy = -(y - self.origin_y)
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def publish_obstacles(self):
        # --- L'ego detenu est-il toujours vivant ? ---
        # Meme garde que dans les deux autres noeuds. Ici l'enjeu est
        # le critere de securite : des obstacles publies dans un
        # repere decale de 31 m rendraient la penalisation de securite
        # arbitraire, sans qu'aucun message ne le signale.
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
            self.origin_x = vehicle_location.x
            self.origin_y = vehicle_location.y
            self.origin_yaw = math.radians(-ego_transform.rotation.yaw)
            self.get_logger().info(
                f"Origine CARLA->map capturee : "
                f"({self.origin_x:.2f}, {self.origin_y:.2f}, "
                f"cap={math.degrees(self.origin_yaw):.1f}deg)"
            )

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        count = 0

        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.id == self.ego_vehicle.id:
                continue
            loc = actor.get_location()
            dist = math.hypot(
                loc.x - vehicle_location.x,
                loc.y - vehicle_location.y
            )
            if dist > self.detection_radius:
                continue
            pose = Pose()
            pose.position.x, pose.position.y = self._carla_to_map(loc.x, loc.y)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            count += 1

        for actor in self.world.get_actors().filter('walker.pedestrian.*'):
            loc = actor.get_location()
            dist = math.hypot(
                loc.x - vehicle_location.x,
                loc.y - vehicle_location.y
            )
            if dist > self.detection_radius:
                continue
            pose = Pose()
            pose.position.x, pose.position.y = self._carla_to_map(loc.x, loc.y)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            count += 1

        self.obstacles_pub.publish(pose_array)

        self.get_logger().info(
            f"{count} obstacles detectes et publies"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
