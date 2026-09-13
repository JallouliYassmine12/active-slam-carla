#!/usr/bin/env python3

import carla
import numpy as np
import math
import rclpy
import os
import signal
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from carla_msgs.msg import CarlaEgoVehicleControl
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header, String
from carla_sensors_bridge.bridge_node import WEATHER_PRESETS

def carla_rotation_to_quaternion(rotation):
    # CARLA utilise un repère main gauche (Unreal), ROS un repère main droite.
    # Comme la position Y est inversée, il faut aussi inverser pitch et yaw
    # pour que l'orientation reste cohérente avec la position.
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
        self.shutting_down = False
        # Lire la variable d'environnement CARLA_HOST
        default_host = os.environ.get('CARLA_HOST', '127.0.0.1')
        default_port = int(os.environ.get('CARLA_PORT', 2000))

        self.declare_parameter('carla_host', default_host)
        self.declare_parameter('carla_port', default_port)
        # Aligne sur la valeur par defaut du fichier de lancement.
        # 'Town03' (carte pleine) provoquerait un rechargement vers
        # une carte qui fait tomber cette configuration GPU des que
        # le noeud est lance seul, sans le launch.
        self.declare_parameter('town', 'Town03_Opt')
        self.declare_parameter('spawn_index', 0)
        self.declare_parameter('weather', 'clear')
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
        self.camera_width = int(
            self.get_parameter('camera_width').value)
        self.camera_height = int(
            self.get_parameter('camera_height').value)
        self.get_logger().info(
            f"RESOLUTION CAMERA : {self.camera_width} x "
            f"{self.camera_height}")
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        town = self.get_parameter('town').value
        self.spawn_index = self.get_parameter('spawn_index').value
        self.weather_name = str(
            self.get_parameter('weather').value
        ).lower()
        self.get_logger().info(f"✅ Connexion à CARLA sur {host}:{port}...")

        self.client = carla.Client(host, port)
        # Timeout genereux : le chargement d'une carte peut prendre plusieurs
        # dizaines de secondes.
        self.client.set_timeout(60.0)

        # --- Chargement de la ville demandee ---
        # Le parametre 'town' etait auparavant lu puis ignore : le bridge
        # travaillait sur la carte deja presente dans le serveur CARLA, quelle
        # qu'elle soit. Deux runs d'une meme campagne comparative pouvaient
        # donc se derouler sur deux villes differentes, ce qui invalide toute
        # comparaison entre strategies. Le sujet exige des scenarios
        # reproductibles : la carte fait partie du scenario.
        #
        # load_world() detruit tous les acteurs et renvoie un NOUVEL objet
        # monde. Il doit donc intervenir AVANT le nettoyage defensif et avant
        # la configuration du mode synchrone ci-dessous, sinon on configurerait
        # un monde sur le point de disparaitre.
        current_map = self.client.get_world().get_map().name.split('/')[-1]

        # --- Comparaison STRICTE ---
        # La condition precedente etait :
        #   if town and current_map != town \
        #           and current_map != f"{town}_Opt":
        # Elle acceptait la variante _Opt comme equivalente a la carte
        # demandee. Lancer town:=Town03 alors que Town03_Opt etait
        # chargee ne declenchait donc aucun rechargement : le run se
        # deroulait sur Town03_Opt en croyant mesurer sur Town03.
        #
        # Les deux cartes ne portent pas le meme maillage de navigation
        # pietonne -- c'est precisement ce qui distingue le scenario
        # pietons exige par le sujet, et la raison pour laquelle
        # num_walkers est force a 0 sur la variante _Opt. La
        # substitution n'etait donc pas anodine, et elle etait
        # silencieuse.
        if town and current_map != town:
            # Les cartes PLEINES chargent tous leurs decors d'un coup,
            # donc des milliers de shaders a compiler. Releve sur cette
            # configuration : LowLevelFatalError "Shader compilation
            # failures are Fatal" au chargement de Town01 comme de
            # Town03. Les variantes _Opt chargent par couches et
            # passent. On avertit, mais on charge quand meme : c'est un
            # choix de scenario, pas une erreur.
            if not town.endswith('_Opt'):
                self.get_logger().warn(
                    f"'{town}' est une carte PLEINE (non _Opt). Sur "
                    f"cette configuration GPU, ces cartes provoquent "
                    f"un LowLevelFatalError de compilation de shaders. "
                    f"La variante '{town}_Opt' charge par couches et "
                    f"passe. Chargement demande malgre tout."
                )
            self.get_logger().info(
                f"Carte actuelle '{current_map}', chargement de "
                f"'{town}'..."
            )
            self.world = self.client.load_world(town)
        else:
            self.get_logger().info(f"Carte '{current_map}' deja chargee.")
            self.world = self.client.get_world()

        # --- Carte REELLEMENT active ---
        # Relue apres coup, pas deduite du parametre. Cette ligne est
        # la trace qui permet de verifier, journal en main, sur quelle
        # carte un run s'est deroule : la carte fait partie du
        # scenario, ce n'est pas un detail de demarrage.
        carte_effective = self.world.get_map().name.split('/')[-1]
        if town and carte_effective != town:
            self.get_logger().error(
                f"CARTE INCORRECTE : '{town}' demandee, "
                f"'{carte_effective}' active. Le scenario mesure "
                f"n'est pas celui demande."
            )
        else:
            self.get_logger().info(f"CARTE ACTIVE : {carte_effective}")

        self.client.set_timeout(10.0)

        self.bp_lib = self.world.get_blueprint_library()

        # --- Meteo ---
        # Prereglages partages avec le pont du SLAM classique (voir
        # l'import de WEATHER_PRESETS en tete de fichier) : RTAB-Map
        # detecte ses fermetures de boucle sur la camera, donc un
        # eclairage different d'un systeme a l'autre fausserait la
        # comparaison entre eux.
        #
        # La valeur par defaut 'clear' vaut carla.WeatherParameters.
        # ClearNoon, exactement ce qui etait applique en dur ici
        # jusqu'a present : les runs de la campagne comparative deja
        # realises restent donc reproductibles a l'identique.
        #
        #   ros2 launch ... weather:=rain
        if self.weather_name not in WEATHER_PRESETS:
            self.get_logger().warn(
                f"Prereglage meteo inconnu : '{self.weather_name}'. "
                f"Valeurs possibles : "
                f"{', '.join(sorted(WEATHER_PRESETS))}. "
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

        # --- Nettoyage defensif d'un run precedent ---
        # Le serveur CARLA persiste entre deux `ros2 launch`, contrairement
        # aux noeuds ROS. Un run interrompu par Ctrl+C laisse le serveur en
        # mode synchrone (fige, en attente d'un world.tick() qui ne viendra
        # plus) et des acteurs orphelins. On repasse d'abord en asynchrone,
        # sinon les destroy() ci-dessous peuvent rester bloques.
        reset_settings = self.world.get_settings()
        reset_settings.synchronous_mode = False
        reset_settings.fixed_delta_seconds = None
        self.world.apply_settings(reset_settings)

        removed = 0
        for pattern in ('sensor.*', 'vehicle.*', 'walker.*'):
            for actor in list(self.world.get_actors().filter(pattern)):
                try:
                    actor.destroy()
                    removed += 1
                except RuntimeError:
                    pass

        if removed:
            self.get_logger().info(
                f"{removed} acteurs d'un run precedent nettoyes"
            )

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05

        self.world.apply_settings(settings)

        # Le traffic manager doit etre synchronise par le client qui appelle
        # world.tick() (ce noeud). S'il est configure depuis un autre client
        # (par exemple traffic_spawner), il ne recoit jamais ses ticks et les
        # vehicules NPC restent figes, meme au feu vert.
        self.traffic_manager = self.client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(True)

        self.get_logger().info(
            f"DEBUG synchronous_mode={settings.synchronous_mode}, "
            f"fixed_delta_seconds={settings.fixed_delta_seconds}"
        )

        self.get_logger().info(
            f"DEBUG no_rendering_mode={settings.no_rendering_mode}"
        )
        # --- Publishers ---
        self.lidar_pub = self.create_publisher(PointCloud2, '/carla/ego/lidar/points', 10)
        self.imu_pub = self.create_publisher(Imu, '/carla/ego/imu', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/carla/ego/gnss', 10)
        self.image_pub = self.create_publisher(Image, '/carla/ego/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/carla/ego/camera/depth', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/carla/ego/camera/camera_info', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # Evenements de collision : le sujet impose d'evaluer le risque de
        # collision (vehicules, pietons, obstacles). Le message porte la
        # famille de l'acteur percute, comptee ensuite par decision_maker.
        self.collision_pub = self.create_publisher(String, '/carla/ego/collision', 10)
        self._last_collision = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_tf_broadcaster = TransformBroadcaster(self)
        # Subscriber pour les commandes de contrôle
        self.cmd_sub = self.create_subscription(CarlaEgoVehicleControl, '/carla/ego/vehicle/vehicle_control_cmd', self.cmd_callback, 10)
        self.get_logger().info("DEBUG Subscriber pour /carla/ego/vehicle/cmd créé")

        self.spawn_vehicle_and_sensors()

        self.get_logger().info("DEBUG spawn_vehicle_and_sensors() termine, lancement du timer odometrie")

        self.timer = self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info("DEBUG init termine, entree dans rclpy.spin() imminente")

    def cmd_callback(self, msg):
        """Callback pour les commandes de contrôle du véhicule"""
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
        self.get_logger().info(
            f"CONTROL reçu throttle={control.throttle}, brake={control.brake}, reverse={control.reverse}"
        )

        self.get_logger().debug(f"Commande reçue: throttle={control.throttle:.2f}, steer={control.steer:.2f}, brake={control.brake:.2f}")

    def spawn_vehicle_and_sensors(self):
        vehicle_bp = self.bp_lib.filter('vehicle.lincoln.mkz_2020')[0]
        vehicle_bp.set_attribute('role_name', 'ego')
        spawn_points = self.world.get_map().get_spawn_points()

        if not spawn_points:
            self.get_logger().error("Aucun point de spawn disponible dans la carte.")
            raise RuntimeError("No spawn points available")

        if self.spawn_index < 0 or self.spawn_index >= len(spawn_points):
            self.get_logger().error(
                f"spawn_index={self.spawn_index} invalide. "
                f"La carte contient {len(spawn_points)} points de spawn."
            )
            raise RuntimeError("Invalid spawn_index")

        sp = spawn_points[self.spawn_index]

        self.vehicle = self.world.try_spawn_actor(vehicle_bp, sp)

        if self.vehicle is not None:
            self.get_logger().info(
                f"Véhicule spawné au point {self.spawn_index} "
                f"(x={sp.location.x:.1f}, y={sp.location.y:.1f})"
            )

        if self.vehicle is None:
            self.get_logger().error("Impossible de spawner le véhicule.")
            raise RuntimeError("Spawn failed")
        self.vehicle.set_autopilot(False)
        self.vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            brake=0.0,
            hand_brake=False
        ))

        # LiDAR
        self.get_logger().info("DEBUG Spawn LiDAR...")
        lidar_bp = self.bp_lib.filter('sensor.lidar.ray_cast')[0]
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '80')
        lidar_bp.set_attribute('points_per_second', '300000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('upper_fov', '15')
        lidar_bp.set_attribute('lower_fov', '-25')
        lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
        self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        self.get_logger().info("DEBUG LiDAR spawne, attachement du listener...")
        self.lidar.listen(self.lidar_callback)
        self.get_logger().info("DEBUG LiDAR OK")

        # IMU
        self.get_logger().info("DEBUG Spawn IMU...")
        imu_bp = self.bp_lib.filter('sensor.other.imu')[0]
        self.imu = self.world.spawn_actor(imu_bp, carla.Transform(), attach_to=self.vehicle)
        self.imu.listen(self.imu_callback)
        self.get_logger().info("DEBUG IMU OK")

        # GNSS
        self.get_logger().info("DEBUG Spawn GNSS...")
        gnss_bp = self.bp_lib.filter('sensor.other.gnss')[0]
        self.gnss = self.world.spawn_actor(gnss_bp, carla.Transform(), attach_to=self.vehicle)
        self.gnss.listen(self.gnss_callback)
        self.get_logger().info("DEBUG GNSS OK")

        # Caméra RGB
        self.get_logger().info("DEBUG Spawn camera RGB...")
        camera_bp = self.bp_lib.filter('sensor.camera.rgb')[0]
        camera_bp.set_attribute('image_size_x', str(self.camera_width))
        camera_bp.set_attribute('image_size_y', str(self.camera_height))
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', '0.1')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.camera_callback)
        self.get_logger().info("DEBUG Camera RGB OK")

        # Paramètres intrinsèques de la caméra
        image_w = self.camera_width
        image_h = self.camera_height
        fov = 90.0
        focal = image_w / (2.0 * math.tan(fov * math.pi / 360.0))
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = image_w
        self.camera_info_msg.height = image_h
        self.camera_info_msg.k = [focal, 0.0, image_w / 2.0,
                                   0.0, focal, image_h / 2.0,
                                   0.0, 0.0, 1.0]
        self.camera_info_msg.p = [focal, 0.0, image_w / 2.0, 0.0,
                                   0.0, focal, image_h / 2.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0]

        # Caméra de profondeur
        self.get_logger().info("DEBUG Spawn camera depth...")
        depth_bp = self.bp_lib.filter('sensor.camera.depth')[0]
        depth_bp.set_attribute('image_size_x', str(self.camera_width))
        depth_bp.set_attribute('image_size_y', str(self.camera_height))
        depth_bp.set_attribute('fov', '90')
        depth_bp.set_attribute('sensor_tick', '0.1')
        depth_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.depth_camera = self.world.spawn_actor(depth_bp, depth_transform, attach_to=self.vehicle)
        self.depth_camera.listen(self.depth_callback)
        self.get_logger().info("DEBUG Spawn capteur de collision...")
        collision_bp = self.bp_lib.find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(
            collision_bp, carla.Transform(), attach_to=self.vehicle
        )
        self.collision_sensor.listen(self.collision_callback)
        self.get_logger().info("DEBUG Camera depth OK - tous les capteurs sont prets")

    def collision_callback(self, event):
        # CARLA declenche ce capteur a chaque tick tant que le contact dure :
        # un frottement de 3 s produirait 60 evenements. On ne compte donc
        # qu'un evenement par acteur percute et par fenetre de 2 secondes.
        if self.shutting_down:
            return
        other = event.other_actor
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_collision.get(other.id, -1e9) < 2.0:
            return
        self._last_collision[other.id] = now
        type_id = other.type_id or ''
        if type_id.startswith('vehicle.'):
            famille = 'vehicule'
        elif type_id.startswith('walker.'):
            famille = 'pieton'
        else:
            famille = 'obstacle'
        msg = String()
        msg.data = famille
        self.collision_pub.publish(msg)
        self.get_logger().warn(f"COLLISION ({famille}) avec {type_id}")

    def lidar_callback(self, point_cloud):
        if self.shutting_down:
            return
        data = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
        points = data[:, :3].tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'ego_lidar'

        msg = point_cloud2.create_cloud_xyz32(header, points)
        self.lidar_pub.publish(msg)

    def imu_callback(self, data):
        if self.shutting_down:
            return
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
        if self.shutting_down:
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_gnss'
        msg.latitude = data.latitude
        msg.longitude = data.longitude
        msg.altitude = data.altitude
        self.gnss_pub.publish(msg)

    def camera_callback(self, image):
        if self.shutting_down:
            return
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
        if self.shutting_down:
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        b = array[:, :, 0].astype(np.float32)
        g = array[:, :, 1].astype(np.float32)
        r = array[:, :, 2].astype(np.float32)
        normalized = (r + g * 256.0 + b * 256.0 * 256.0) / (256.0 * 256.0 * 256.0 - 1.0)
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

    def publish_odometry(self):
        if self.shutting_down:
            return

        self.world.tick()

        if self.vehicle is None:
            return

        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()

        # Faire suivre la caméra spectateur CARLA au véhicule (vue troisième personne, en hauteur)
        spectator = self.world.get_spectator()
        vehicle_transform = self.vehicle.get_transform()
        spectator_location = vehicle_transform.location + carla.Location(
            x=-8 * math.cos(math.radians(vehicle_transform.rotation.yaw)),
            y=-8 * math.sin(math.radians(vehicle_transform.rotation.yaw)),
            z=4
        )
        spectator_rotation = carla.Rotation(
            pitch=-15,
            yaw=vehicle_transform.rotation.yaw,
            roll=0
        )
        spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))

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
        # NE JAMAIS publier une transformee dont l'enfant est 'base_link' :
        # icp_odometry publie deja odom_visual -> base_link (publish_tf=True
        # dans le launch). Un repere TF ne peut avoir qu'UN SEUL parent.
        # Deux emetteurs concurrents scindent l'arbre en deux, RTAB-Map ne
        # peut plus relier odom_visual a base_link ("Tf has two or more
        # unconnected trees"), sa covariance grimpe sans contrainte, et le
        # run s'arrete sur incertitude critique au bout d'une minute.
        #
        # La position vraie fournie par CARLA est donc publiee dans son
        # propre arbre map_gt -> base_link_gt. Elle reste affichable dans
        # RViz a cote de la trajectoire SLAM, sans interferer avec elle.
        # L'evaluateur de localisation lit de toute facon le topic Odometry
        # publie juste au-dessus, pas la TF.
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
        # contradictoires pour la meme arete faisaient osciller la camera de
        # 20 cm a chaque cycle, ce qui degrade l'appariement visuel de
        # RTAB-Map et donc la detection de fermetures de boucle.

    def destroy(self):
        # Idempotent : cette methode est appelee une premiere fois par le
        # gestionnaire de signal (avant que rclpy ne commence a finaliser
        # l'interpreteur) puis une seconde fois dans le bloc finally de main().
        if self.shutting_down:
            return
        self.shutting_down = True

        # 1. Couper le timer d'odometrie AVANT tout : plus aucun world.tick()
        #    ne peut demarrer. Un tick en cours bloque jusqu'au timeout client,
        #    ce qui empeche le noeud de repondre au SIGINT et force ros2 launch
        #    a l'abattre au SIGKILL.
        try:
            if getattr(self, 'timer', None) is not None:
                self.timer.cancel()
        except Exception:
            pass

        # 2. Arreter les capteurs. Leurs callbacks tournent dans des threads
        #    CARLA et continueraient sinon a publier pendant la finalisation
        #    de l'interpreteur Python, ce qui provoque le
        #    "Fatal Python error: PyGILState_Release" et un arret en SIGABRT.
        for name in ('camera', 'depth_camera', 'lidar', 'imu', 'gnss', 'collision_sensor'):
            sensor = getattr(self, name, None)
            if sensor is not None:
                try:
                    sensor.stop()
                except RuntimeError:
                    pass

        # 3. Timeout court : les appels reseau qui suivent ne doivent pas
        #    immobiliser l'arret pendant 10 secondes si le serveur ne repond
        #    pas immediatement.
        try:
            self.client.set_timeout(2.0)
        except RuntimeError:
            pass

        # 4. Restaure le mode asynchrone : sans ca, le serveur CARLA reste fige
        #    a attendre un world.tick() qui ne viendra plus, et le run suivant
        #    demarre sur un simulateur bloque.
        try:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
        except RuntimeError:
            pass

        try:
            if getattr(self, 'traffic_manager', None) is not None:
                self.traffic_manager.set_synchronous_mode(False)
        except RuntimeError:
            pass

        # 5. Destruction des acteurs.
        for name in ('camera', 'depth_camera', 'lidar', 'imu', 'gnss', 'vehicle'):
            actor = getattr(self, name, None)
            if actor is not None:
                try:
                    actor.destroy()
                except RuntimeError:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = CarlaSensorsBridge()

    # Les capteurs CARLA appellent leurs callbacks depuis des threads C++.
    # Si l'interpreteur Python commence a se finaliser (arret de rclpy) alors
    # qu'un callback est encore en cours, on obtient
    # "Fatal Python error: PyGILState_Release" et le processus doit etre tue
    # de force -- laissant le serveur CARLA fige en mode synchrone.
    # On coupe donc les capteurs des la RECEPTION du signal, avant que rclpy
    # n'ait commence quoi que ce soit, puis on rend la main au gestionnaire
    # d'origine pour que l'arret ROS se deroule normalement.
    previous_handlers = {}

    def _shutdown_handler(signum, frame):
        try:
            node.destroy()
        except Exception:
            pass
        previous = previous_handlers.get(signum)
        if callable(previous):
            previous(signum, frame)
        else:
            raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            previous_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, _shutdown_handler)
        except (ValueError, OSError):
            pass

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
