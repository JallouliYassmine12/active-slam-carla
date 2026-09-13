#!/usr/bin/env python3
"""
Pont CARLA <-> ROS 2 pour le systeme de reference (SLAM classique).

Ce noeud spawne le vehicule ego et ses capteurs, publie les donnees vers
ROS 2, et applique les commandes de controle recues.

Trois corrections importantes par rapport a la version precedente :

  1. Point de spawn deterministe. La version precedente tirait le point
     de depart au hasard (random.shuffle). Deux consequences : les runs
     n'etaient pas reproductibles, ce que la tache 3 du sujet exige
     explicitement, et la comparaison avec l'Active SLAM etait faussee
     puisque celui-ci part toujours du meme endroit. Un run parti d'un
     quartier dense n'affronte pas le meme probleme qu'un run parti
     d'une avenue rectiligne.

  2. Restauration du mode synchrone a l'arret. Le noeud passait CARLA en
     mode synchrone sans jamais l'en sortir. A la fermeture, le
     simulateur restait bloque en attente d'un tick qui ne venait plus,
     et tout appel client suivant expirait :
         terminate called after throwing an instance of
         'carla::client::TimeoutException'
     C'est exactement l'erreur observee a la fin des runs.

  3. Meteo parametrable et fixee pour toute la duree du run (tache 4 du
     sujet). Voir la note sur la portee reelle de la meteo dans
     apply_weather().
"""

import carla
import numpy as np
import math
import random
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from carla_msgs.msg import CarlaEgoVehicleControl
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header


# ==========================================================
# Prereglages meteo
# ==========================================================
#
# Les noms sont ceux du sujet, pas ceux de CARLA, pour que la ligne de
# commande reste lisible :
#     ros2 launch ... weather:=rain
#
# 'fog' et 'night' n'existent pas comme prereglages CARLA : ils sont
# construits explicitement.

def _weather_fog():
    w = carla.WeatherParameters.CloudyNoon
    w.fog_density = 80.0
    w.fog_distance = 10.0
    w.fog_falloff = 1.0
    return w


def _weather_night():
    w = carla.WeatherParameters.ClearNoon
    w.sun_altitude_angle = -25.0
    return w


WEATHER_PRESETS = {
    'clear': lambda: carla.WeatherParameters.ClearNoon,
    'cloudy': lambda: carla.WeatherParameters.CloudyNoon,
    'wet': lambda: carla.WeatherParameters.WetNoon,
    'light_rain': lambda: carla.WeatherParameters.SoftRainNoon,
    'rain': lambda: carla.WeatherParameters.HardRainNoon,
    'sunset': lambda: carla.WeatherParameters.ClearSunset,
    'fog': _weather_fog,
    'night': _weather_night,
}


def carla_rotation_to_quaternion(rotation):
    # CARLA utilise un repere main gauche (Unreal), ROS un repere main
    # droite. Comme la position Y est inversee, il faut aussi inverser
    # pitch et yaw pour que l'orientation reste coherente avec la
    # position.
    roll = math.radians(rotation.roll)
    pitch = math.radians(-rotation.pitch)
    yaw = math.radians(-rotation.yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class CarlaSensorsBridge(Node):

    def __init__(self):
        super().__init__('carla_sensors_bridge')

        default_host = os.environ.get('CARLA_HOST', '127.0.0.1')
        default_port = int(os.environ.get('CARLA_PORT', 2000))

        self.declare_parameter('carla_host', default_host)
        self.declare_parameter('carla_port', default_port)
        self.declare_parameter('role_name', 'ego')

        # --- Reproductibilite du point de depart ---
        # spawn_point_index fixe le point de depart. La liste renvoyee
        # par get_spawn_points() est stable pour une carte donnee, donc
        # l'index designe toujours le meme endroit. C'est ce qui rend le
        # scenario reproductible et comparable a l'Active SLAM, qui part
        # du point 0.
        self.declare_parameter('spawn_point_index', 0)
        # random_spawn reintroduit le tirage aleatoire si on veut
        # explicitement tester la robustesse sur des departs varies.
        # C'est alors un choix assume, pas un effet de bord.
        self.declare_parameter('random_spawn', False)
        self.declare_parameter('random_seed', 42)

        # --- Meteo (tache 4 du sujet) ---
        self.declare_parameter('weather', 'clear')

        # --- Carte ---
        # 'town' etait declare et lu dans la version precedente, puis
        # jamais utilise : le run tournait donc sur la carte deja
        # ouverte dans CARLA, quelle qu'elle soit, alors que le fichier
        # de lancement affichait 'Town01'. On ne charge la carte que si
        # load_town est vrai (le chargement de certaines cartes fait
        # planter la compilation des shaders sur certains GPU), mais on
        # journalise toujours la carte reellement chargee.
        self.declare_parameter('town', '')
        self.declare_parameter('load_town', False)

        # --- Resolution des cameras RGB et profondeur ---
        # 320 x 240 = valeur de tous les runs archives : sans
        # variable d'environnement, le comportement est inchange.
        #
        # Elle est exposee parce que la transformation d'une
        # fermeture de boucle est estimee visuellement avant d'etre
        # affinee par ICP (Reg/Strategy = 2). A 320 x 240 sur des
        # facades urbaines repetitives, cette estimation echoue et
        # la fermeture est rejetee faute de transformation
        # calculable -- 2 acceptees sur 129 proposees, mesure sur
        # quinze runs.
        #
        #   CAMERA_WIDTH=640 CAMERA_HEIGHT=480 ros2 launch ...
        self.declare_parameter(
            'camera_width', int(os.environ.get('CAMERA_WIDTH', 320)))
        self.declare_parameter(
            'camera_height', int(os.environ.get('CAMERA_HEIGHT', 240)))

        host = self.get_parameter('carla_host').value
        port = int(self.get_parameter('carla_port').value)
        self.role_name = self.get_parameter('role_name').value
        self.spawn_point_index = int(
            self.get_parameter('spawn_point_index').value
        )
        self.random_spawn = bool(self.get_parameter('random_spawn').value)
        self.random_seed = int(self.get_parameter('random_seed').value)
        self.weather_name = str(self.get_parameter('weather').value).lower()
        self.town = str(self.get_parameter('town').value)
        self.load_town = bool(self.get_parameter('load_town').value)
        self.camera_width = int(
            self.get_parameter('camera_width').value)
        self.camera_height = int(
            self.get_parameter('camera_height').value)
        self.get_logger().info(
            f"RESOLUTION CAMERA : {self.camera_width} x "
            f"{self.camera_height}")

        self.get_logger().info(f"Connexion a CARLA sur {host}:{port} ...")

        self.client = carla.Client(host, port)
        self.client.set_timeout(20.0)

        if self.load_town and self.town:
            self.get_logger().info(f"Chargement de la carte '{self.town}' ...")
            self.world = self.client.load_world(self.town)
        else:
            self.world = self.client.get_world()

        # On journalise la carte reellement active : c'est cette ligne
        # qui fera foi dans le rapport, pas le parametre demande.
        carte_active = self.world.get_map().name
        self.get_logger().info(f"CARTE ACTIVE : {carte_active}")

        if self.town and self.town not in carte_active:
            self.get_logger().warn(
                f"La carte demandee ('{self.town}') n'est PAS celle qui est "
                f"chargee ('{carte_active}'). Le run se deroule sur "
                f"'{carte_active}'. Utiliser load_town:=true pour forcer le "
                f"chargement."
            )

        self.bp_lib = self.world.get_blueprint_library()

        # On memorise les reglages d'origine pour pouvoir les restaurer
        # a l'arret (voir destroy()).
        self.original_settings = self.world.get_settings()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        self.get_logger().info(
            f"Mode synchrone actif, pas de temps fixe = "
            f"{settings.fixed_delta_seconds} s"
        )

        self.apply_weather()

        # --- Publishers ---
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/carla/ego/lidar/points', 10)
        self.imu_pub = self.create_publisher(Imu, '/carla/ego/imu', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/carla/ego/gnss', 10)
        self.image_pub = self.create_publisher(
            Image, '/carla/ego/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, '/carla/ego/camera/depth', 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/carla/ego/camera/camera_info', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            CarlaEgoVehicleControl,
            '/carla/ego/vehicle/vehicle_control_cmd',
            self.cmd_callback,
            10
        )

        self.spawn_vehicle_and_sensors()

        self.timer = self.create_timer(0.05, self.publish_odometry)

    # ======================================================
    # Meteo
    # ======================================================

    def apply_weather(self):
        """
        Applique un prereglage meteo, fixe pour toute la duree du run.

        Portee reelle de la meteo dans cette chaine, a mentionner
        explicitement dans le rapport :

          - La camera RGB est affectee (pluie, brouillard, luminosite).
            C'est elle qui alimente la detection de fermeture de boucle
            de RTAB-Map, via les descripteurs visuels. La meteo degrade
            donc la capacite du systeme a reconnaitre un lieu deja
            visite.
          - Le LiDAR 'ray_cast' de CARLA n'est PAS attenue par la meteo :
            ses seules sources de bruit sont ses propres parametres. Or
            c'est lui qui alimente icp_odometry.

        Autrement dit, dans cette architecture, la meteo degrade la
        fermeture de boucle mais pas l'odometrie. C'est une limite du
        simulateur, pas du systeme, et il vaut mieux l'ecrire que la
        laisser decouvrir.
        """
        if self.weather_name not in WEATHER_PRESETS:
            self.get_logger().warn(
                f"Prereglage meteo inconnu : '{self.weather_name}'. "
                f"Valeurs admises : {', '.join(sorted(WEATHER_PRESETS))}. "
                f"Repli sur 'clear'."
            )
            self.weather_name = 'clear'

        weather = WEATHER_PRESETS[self.weather_name]()
        self.world.set_weather(weather)

        self.get_logger().info(
            f"METEO : {self.weather_name} "
            f"(pluie={weather.precipitation:.0f}, "
            f"brouillard={weather.fog_density:.0f}, "
            f"soleil={weather.sun_altitude_angle:.0f} deg)"
        )

    # ======================================================
    # Controle
    # ======================================================

    def cmd_callback(self, msg):
        if self.vehicle is None:
            return

        control = carla.VehicleControl()
        control.throttle = float(msg.throttle)
        control.steer = float(msg.steer)
        control.brake = float(msg.brake)
        control.reverse = bool(msg.reverse)
        control.hand_brake = bool(msg.hand_brake)
        control.manual_gear_shift = False
        control.gear = 1

        self.vehicle.apply_control(control)

        # Journalisation en debug et non en info : la version precedente
        # ecrivait une ligne a chaque commande, soit 10 a 20 lignes par
        # seconde, ce qui noyait les messages utiles dans le log du run
        # et grossissait inutilement les fichiers analyses ensuite.
        self.get_logger().debug(
            f"Commande : throttle={control.throttle:.2f} "
            f"steer={control.steer:.2f} brake={control.brake:.2f}"
        )

    # ======================================================
    # Spawn
    # ======================================================

    def spawn_vehicle_and_sensors(self):
        vehicle_bp = self.bp_lib.filter('vehicle.lincoln.mkz_2020')[0]
        vehicle_bp.set_attribute('role_name', self.role_name)

        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            raise RuntimeError("La carte ne fournit aucun point de spawn.")

        if self.random_spawn:
            # Tirage aleatoire, mais avec une graine fixe : le run reste
            # reproductible a l'identique d'un lancement a l'autre.
            random.Random(self.random_seed).shuffle(spawn_points)
            ordre = spawn_points
            self.get_logger().info(
                f"Spawn aleatoire active (graine={self.random_seed})"
            )
        else:
            # Point impose en premier, puis les autres en repli si celui
            # demande est occupe.
            idx = self.spawn_point_index % len(spawn_points)
            ordre = [spawn_points[idx]] + [
                sp for i, sp in enumerate(spawn_points) if i != idx
            ]

        self.vehicle = None

        for i, sp in enumerate(ordre):
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, sp)

            if self.vehicle is not None:
                if i == 0:
                    origine = (
                        f"point demande {self.spawn_point_index}"
                        if not self.random_spawn
                        else f"premier point du tirage (graine={self.random_seed})"
                    )
                else:
                    origine = f"repli n°{i} (le point demande etait occupe)"

                self.get_logger().info(
                    f"VEHICULE SPAWNE : {origine} | "
                    f"x={sp.location.x:.2f}, y={sp.location.y:.2f}, "
                    f"cap={sp.rotation.yaw:.1f} deg"
                )
                break

        if self.vehicle is None:
            raise RuntimeError("Impossible de spawner le vehicule ego.")

        self.vehicle.set_autopilot(False)
        self.vehicle.apply_control(
            carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False)
        )

        # --- LiDAR ---
        lidar_bp = self.bp_lib.filter('sensor.lidar.ray_cast')[0]
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '80')
        lidar_bp.set_attribute('points_per_second', '300000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('upper_fov', '15')
        lidar_bp.set_attribute('lower_fov', '-25')

        self.lidar = self.world.spawn_actor(
            lidar_bp,
            carla.Transform(carla.Location(x=0, z=2.5)),
            attach_to=self.vehicle
        )
        self.lidar.listen(self.lidar_callback)

        # --- IMU ---
        imu_bp = self.bp_lib.filter('sensor.other.imu')[0]
        self.imu = self.world.spawn_actor(
            imu_bp, carla.Transform(), attach_to=self.vehicle)
        self.imu.listen(self.imu_callback)

        # --- GNSS ---
        gnss_bp = self.bp_lib.filter('sensor.other.gnss')[0]
        self.gnss = self.world.spawn_actor(
            gnss_bp, carla.Transform(), attach_to=self.vehicle)
        self.gnss.listen(self.gnss_callback)

        # --- Camera RGB ---
        camera_bp = self.bp_lib.filter('sensor.camera.rgb')[0]
        camera_bp.set_attribute('image_size_x', str(self.camera_width))
        camera_bp.set_attribute('image_size_y', str(self.camera_height))
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', '0.1')

        self.camera = self.world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=1.5, z=2.4)),
            attach_to=self.vehicle
        )
        self.camera.listen(self.camera_callback)

        # Parametres intrinseques
        image_w, image_h, fov = (
            self.camera_width, self.camera_height, 90.0)
        focal = image_w / (2.0 * math.tan(fov * math.pi / 360.0))

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = image_w
        self.camera_info_msg.height = image_h
        self.camera_info_msg.k = [
            focal, 0.0, image_w / 2.0,
            0.0, focal, image_h / 2.0,
            0.0, 0.0, 1.0,
        ]
        self.camera_info_msg.p = [
            focal, 0.0, image_w / 2.0, 0.0,
            0.0, focal, image_h / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        # --- Camera de profondeur ---
        depth_bp = self.bp_lib.filter('sensor.camera.depth')[0]
        depth_bp.set_attribute('image_size_x', str(self.camera_width))
        depth_bp.set_attribute('image_size_y', str(self.camera_height))
        depth_bp.set_attribute('fov', '90')
        depth_bp.set_attribute('sensor_tick', '0.1')

        self.depth_camera = self.world.spawn_actor(
            depth_bp,
            carla.Transform(carla.Location(x=1.5, z=2.4)),
            attach_to=self.vehicle
        )
        self.depth_camera.listen(self.depth_callback)

        self.get_logger().info("Tous les capteurs sont prets.")

    # ======================================================
    # Callbacks capteurs
    # ======================================================

    def lidar_callback(self, point_cloud):
        data = np.frombuffer(
            point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
        points = data[:, :3].tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'ego_lidar'

        self.lidar_pub.publish(
            point_cloud2.create_cloud_xyz32(header, points))

    def imu_callback(self, data):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_imu'
        msg.linear_acceleration.x = data.accelerometer.x
        msg.linear_acceleration.y = data.accelerometer.y
        msg.linear_acceleration.z = data.accelerometer.z
        msg.angular_velocity.x = data.gyroscope.x
        msg.angular_velocity.y = data.gyroscope.y
        msg.angular_velocity.z = data.gyroscope.z
        msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    def gnss_callback(self, data):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_gnss'
        msg.latitude = data.latitude
        msg.longitude = data.longitude
        msg.altitude = data.altitude
        self.gnss_pub.publish(msg)

    def camera_callback(self, image):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_camera'
        msg.height = image.height
        msg.width = image.width
        msg.encoding = 'bgra8'
        msg.step = image.width * 4
        msg.data = bytes(image.raw_data)
        self.image_pub.publish(msg)

    def depth_callback(self, image):
        array = np.frombuffer(
            image.raw_data, dtype=np.uint8
        ).reshape((image.height, image.width, 4))

        b = array[:, :, 0].astype(np.float32)
        g = array[:, :, 1].astype(np.float32)
        r = array[:, :, 2].astype(np.float32)

        normalized = (
            (r + g * 256.0 + b * 256.0 * 256.0)
            / (256.0 * 256.0 * 256.0 - 1.0)
        )
        depth_meters = normalized * 1000.0

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_camera'
        msg.height = image.height
        msg.width = image.width
        msg.encoding = '32FC1'
        msg.step = image.width * 4
        msg.data = depth_meters.astype(np.float32).tobytes()
        self.depth_pub.publish(msg)

        self.camera_info_msg.header.stamp = msg.header.stamp
        self.camera_info_msg.header.frame_id = 'ego_camera'
        self.camera_info_pub.publish(self.camera_info_msg)

    # ======================================================
    # Odometrie de verite terrain
    # ======================================================

    def publish_odometry(self):
        self.world.tick()

        if self.vehicle is None:
            return

        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()

        # Camera spectateur suivant le vehicule (vue troisieme personne)
        spectator = self.world.get_spectator()
        spectator_location = transform.location + carla.Location(
            x=-8 * math.cos(math.radians(transform.rotation.yaw)),
            y=-8 * math.sin(math.radians(transform.rotation.yaw)),
            z=4
        )
        spectator.set_transform(carla.Transform(
            spectator_location,
            carla.Rotation(pitch=-15, yaw=transform.rotation.yaw, roll=0)
        ))

        qx, qy, qz, qw = carla_rotation_to_quaternion(transform.rotation)

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = transform.location.x
        odom.pose.pose.position.y = -transform.location.y
        odom.pose.pose.position.z = transform.location.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = velocity.x
        odom.twist.twist.linear.y = -velocity.y
        odom.twist.twist.linear.z = velocity.z
        self.odom_pub.publish(odom)

        # --- Verite terrain CARLA, dans un arbre TF SEPARE ---
        #
        # NE JAMAIS publier une transformee dont l'enfant est 'base_link' :
        # icp_odometry publie deja odom_visual -> base_link (publish_tf=True
        # dans le launch). Un repere TF ne peut avoir qu'UN SEUL parent.
        # Deux emetteurs concurrents scindent l'arbre en deux, RTAB-Map ne
        # peut plus relier odom_visual a base_link ("Tf has two or more
        # unconnected trees"), et sa covariance grimpe sans contrainte.
        #
        # La version precedente de ce fichier publiait odom -> base_link :
        # le SLAM classique tournait donc avec un arbre TF scinde, ce qui
        # degradait sa localisation pour une raison purement technique,
        # sans aucun rapport avec la methode d'exploration comparee.
        #
        # La position vraie est donc publiee dans son propre arbre
        # map_gt -> base_link_gt, affichable dans RViz a cote de la
        # trajectoire SLAM. L'evaluateur lit de toute facon le topic
        # Odometry publie juste au-dessus, pas la TF.
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map_gt'
        t.child_frame_id = 'base_link_gt'
        t.transform.translation.x = transform.location.x
        t.transform.translation.y = -transform.location.y
        t.transform.translation.z = transform.location.z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # La transformee base_link -> ego_camera n'est PLUS publiee ici :
        # static_transform_publisher_camera la publie deja dans le launch
        # avec z=2.4, alors que ce code emettait z=2.2. Deux valeurs
        # contradictoires pour la meme arete faisaient osciller la camera
        # de 20 cm a chaque cycle, ce qui degrade l'appariement visuel de
        # RTAB-Map et donc la detection de fermetures de boucle.

    # ======================================================
    # Arret propre
    # ======================================================

    def destroy(self):
        """
        Ordre imperatif : arreter les capteurs, puis detruire les
        acteurs, puis RENDRE LE SIMULATEUR AU MODE ASYNCHRONE.

        Sans la derniere etape, CARLA reste en mode synchrone en
        attendant un tick qui ne viendra plus. Tout client suivant
        (y compris le lancement d'apres) bloque alors jusqu'a expiration
        du delai, et le pont meurt sur une TimeoutException.
        """
        for nom in ('camera', 'depth_camera', 'lidar', 'imu', 'gnss'):
            capteur = getattr(self, nom, None)
            if capteur is not None:
                try:
                    capteur.stop()
                except Exception:
                    pass
                try:
                    capteur.destroy()
                except Exception:
                    pass

        vehicule = getattr(self, 'vehicle', None)
        if vehicule is not None:
            try:
                vehicule.destroy()
            except Exception:
                pass

        try:
            if getattr(self, 'original_settings', None) is not None:
                self.world.apply_settings(self.original_settings)
            else:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)

            print('[bridge_node] mode synchrone desactive, simulateur rendu '
                  'a son etat initial.')
        except Exception as exc:  # noqa: BLE001
            print(f'[bridge_node] echec de la restauration des reglages '
                  f'CARLA : {exc}')


def main(args=None):
    rclpy.init(args=args)

    node = CarlaSensorsBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # destroy() utilise print() et non le logger : apres SIGINT le
        # contexte ROS peut deja etre invalide, ce qui provoquait les
        # "Failed to publish log message to rosout".
        node.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
