#!/usr/bin/env python3
"""
Controleur du systeme de reference (SLAM classique) : trajectoire
predefinie suivie en pure pursuit le long du graphe routier CARLA.

C'est la baseline "trajectoire predefinie" exigee par le sujet. Elle
doit differer de l'Active SLAM sur UN SEUL point : la maniere de
choisir ou aller. Tout le reste -- competence de conduite, respect des
feux, gestion des obstacles, loi de commande longitudinale, mesure de
la distance parcourue -- doit etre identique, sinon l'ecart mesure
entre les deux systemes ne dit rien de la methode d'exploration.

Corrections apportees a la version precedente
---------------------------------------------

1. IDENTIFICATION DE L'EGO PAR role_name.
   L'ancienne version prenait max(vehicles, key=id). Avec un seul
   vehicule sur la carte cela fonctionnait par accident. Des que
   traffic_spawner peuple l'environnement, l'identifiant le plus eleve
   est celui du DERNIER NPC cree : le controleur generait alors ses
   waypoints depuis la position d'un NPC et lisait les feux d'un autre
   vehicule.

2. GESTION DES OBSTACLES.
   Cette version n'en avait aucune. Le controleur Active SLAM s'arrete
   a 7 m derriere un obstacle ; celui-ci lui rentrait dedans. Avec du
   trafic des deux cotes, l'ecart de performance aurait mesure une
   difference de competence de conduite, pas de methode d'exploration.
   On reprend donc le meme filtre par COULOIR que le controleur Active
   SLAM (projection dans le repere du vehicule, rejet de ce dont
   l'ecart lateral depasse la demi-largeur), avec les memes seuils.

3. FEUX ROUGES.
   L'ancienne version ne freinait qu'a moins de 0.5 m de la ligne
   d'arret : a 6 m/s le vehicule l'avait deja franchie. Elle engageait
   de plus la marche arriere (cmd.reverse = True) pendant le freinage,
   et ne posait pas waiting_traffic_light, si bien qu'une attente
   normale au feu etait comptee comme un blocage. Freinage progressif
   desormais, a partir de light_slow_distance.

4. ANTI-BLOCAGE REELLEMENT BRANCHE.
   check_stuck() et recovery_maneuver() n'etaient jamais appelees
   depuis control_loop : la banniere annoncait un anti-blocage qui
   n'existait pas. Elles sont maintenant integrees a la boucle, et la
   manoeuvre de degagement fait quelque chose (marche arriere braquee)
   au lieu de commander throttle=0, brake=0, steer=0.

5. RETURN MANQUANT.
   Quand la fin de l'itineraire etait atteinte, la fonction tuait le
   groupe de processus puis continuait son execution jusqu'a target[0]
   sur un target valant None -- TypeError selon le timing du signal.

6. DISTANCE PARCOURUE PUBLIEE sur /vehicle/distance_traveled, comme le
   controleur Active SLAM, pour que le script d'analyse lise la meme
   grandeur sur le meme topic pour les deux systemes.
"""

import math
import os
import signal
import time

import numpy as np

import carla

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl


class VehicleControllerEval(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        # --- Trajectoire et budget ---
        self.declare_parameter('distance', 1500.0)
        self.declare_parameter('lookahead_distance', 8.0)
        self.declare_parameter('max_steering_angle', 0.8)

        # --- Identification de l'ego ---
        self.declare_parameter('role_name', 'ego')
        self.declare_parameter('carla_host', '127.0.0.1')
        self.declare_parameter('carla_port', 2000)

        # --- Loi de commande longitudinale ---
        # Memes valeurs que vehicle_controller_active_slam : la vitesse
        # est une variable de confusion majeure (l'ICP se degrade quand
        # les scans se recouvrent moins), elle doit donc etre identique
        # des deux cotes.
        self.declare_parameter('max_throttle', 0.5)
        self.declare_parameter('min_throttle', 0.35)

        # --- Obstacles : memes seuils que le controleur Active SLAM ---
        self.declare_parameter('obstacle_stop_distance', 7.0)
        self.declare_parameter('obstacle_slow_distance', 14.0)
        self.declare_parameter('obstacle_corridor_width', 3.0)
        self.declare_parameter('obstacle_scan_radius', 60.0)
        self.declare_parameter('obstacle_refresh_rate', 2.0)   # Hz
        # Duree maximale d'attente derriere un obstacle avant de le
        # considerer comme definitif. Meme valeur que le controleur
        # Active SLAM. Sans cette borne, un vehicule NPC gare dans le
        # couloir bloque le run indefiniment : l'attente est marquee
        # "deliberee", donc le chien de garde anti-blocage ne se
        # declenche jamais. Observe sur le run de controle : 17 s
        # d'arret consecutif a 5.6 m d'un obstacle immobile.
        self.declare_parameter('max_obstacle_wait', 60.0)

        # --- Feux et stops ---
        self.declare_parameter('respect_traffic_lights', True)
        self.declare_parameter('light_stop_distance', 3.0)
        self.declare_parameter('light_slow_distance', 12.0)
        self.declare_parameter('stop_sign_distance', 5.0)

        # --- Anti-blocage ---
        self.declare_parameter('stuck_threshold', 50)          # cycles a 10 Hz
        self.declare_parameter('recovery_cycles', 25)

        self.distance = float(self.get_parameter('distance').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)

        self.role_name = self.get_parameter('role_name').value
        self.carla_host = os.environ.get(
            'CARLA_HOST', self.get_parameter('carla_host').value)
        self.carla_port = int(self.get_parameter('carla_port').value)

        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.min_throttle = float(self.get_parameter('min_throttle').value)

        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)
        self.obstacle_slow_distance = float(
            self.get_parameter('obstacle_slow_distance').value)
        self.obstacle_corridor_width = float(
            self.get_parameter('obstacle_corridor_width').value)
        self.obstacle_scan_radius = float(
            self.get_parameter('obstacle_scan_radius').value)
        obstacle_refresh_rate = float(
            self.get_parameter('obstacle_refresh_rate').value)
        self.max_obstacle_wait = float(
            self.get_parameter('max_obstacle_wait').value)

        self.respect_traffic_lights = bool(
            self.get_parameter('respect_traffic_lights').value)
        self.light_stop_distance = float(
            self.get_parameter('light_stop_distance').value)
        self.light_slow_distance = float(
            self.get_parameter('light_slow_distance').value)
        self.stop_sign_distance = float(
            self.get_parameter('stop_sign_distance').value)

        self.stuck_threshold = int(self.get_parameter('stuck_threshold').value)
        self.recovery_cycles = int(self.get_parameter('recovery_cycles').value)

        if self.min_throttle > self.max_throttle:
            self.min_throttle = self.max_throttle

        # --- Etat ---
        self.current_pose = None
        self.current_velocity = 0.0
        self.current_yaw = 0.0
        self.current_wp_index = 0
        self.waypoints = []
        self.distance_traveled = 0.0
        self.last_position = None

        self.stuck_counter = 0
        self.is_recovering = False
        self.recovery_counter = 0
        self.recovery_direction = 0.6
        self._stuck_ref_pos = None

        self.stop_signs = []
        self.stopped_signs_ids = set()
        self.stop_wait_counter = 0
        self.is_stopping_at_sign = False

        self.waiting_deliberately = False   # feu rouge ou stop : pas un blocage
        self.stopped_at_light = False
        self.obstacle_wait_start = None

        self.command_counter = 0
        self.waypoints_ready = False
        self.finished = False

        self.world = None
        self.ego_actor = None
        self.obstacles = []                 # positions (x, -y) repere waypoints
        self.obstacle_refresh_period = (
            1.0 / obstacle_refresh_rate if obstacle_refresh_rate > 0 else 0.5
        )

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego/vehicle/vehicle_control_cmd',
            10
        )
        # Meme topic que le controleur Active SLAM : le script d'analyse
        # lit ainsi la meme grandeur au meme endroit pour les deux systemes.
        self.distance_pub = self.create_publisher(
            Float32, '/vehicle/distance_traveled', 10
        )

        # --- Subscribers ---
        # Un seul abonnement : /odom porte a la fois la position (verite
        # terrain CARLA) et la vitesse, via son twist.
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(
            f"VEHICLE CONTROLLER EVAL (trajectoire predefinie) demarre | "
            f"budget={self.distance:.0f}m | "
            f"throttle max={self.max_throttle:.2f} | "
            f"obstacles : couloir={self.obstacle_corridor_width:.1f}m, "
            f"arret a {self.obstacle_stop_distance:.1f}m | "
            f"feux respectes={self.respect_traffic_lights}"
        )

        self.waypoint_retry_timer = self.create_timer(
            1.0, self._try_generate_waypoints)

    # ------------------------------------------------------------------
    # Demarrage
    # ------------------------------------------------------------------

    def _try_generate_waypoints(self):
        if self.waypoints_ready:
            return
        if self.generate_waypoints():
            self.waypoints_ready = True
            self.waypoint_retry_timer.cancel()
            self.control_timer = self.create_timer(0.1, self.control_loop)
            self.create_timer(
                self.obstacle_refresh_period, self._refresh_obstacles)
            self.get_logger().info(
                'Waypoints prets, demarrage de la boucle de controle.')

    def _find_ego(self, world):
        """L'ego est identifie par son role_name, jamais par son id.

        traffic_spawner cree des dizaines de NPC apres l'ego : le
        vehicule d'identifiant le plus eleve est l'un d'eux, pas l'ego.
        """
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.role_name:
                return actor
        return None

    def generate_waypoints(self):
        """Genere la trajectoire predefinie depuis la position reelle de
        l'ego, en suivant le graphe routier CARLA."""
        self.waypoints = []
        try:
            client = carla.Client(self.carla_host, self.carla_port)
            client.set_timeout(10.0)
            self.world = client.get_world()
            carla_map = self.world.get_map()

            self.stop_signs = carla_map.get_all_landmarks_of_type('206')
            self.stopped_signs_ids = set()

            ego_actor = self._find_ego(self.world)
            if ego_actor is None:
                nb = len(self.world.get_actors().filter('vehicle.*'))
                self.get_logger().warn(
                    f"Vehicule ego (role_name='{self.role_name}') introuvable "
                    f"parmi {nb} vehicules, nouvel essai..."
                )
                return False

            self.ego_actor = ego_actor
            self.get_logger().info(
                f"Ego identifie par role_name : id={ego_actor.id} | "
                f"{len(self.stop_signs)} panneaux stop sur la carte."
            )

            start_transform = ego_actor.get_transform()
            start_location = start_transform.location
            vehicle_yaw = start_transform.rotation.yaw

            wp = carla_map.get_waypoint(start_location)
            wp_yaw = wp.transform.rotation.yaw
            yaw_diff = abs((vehicle_yaw - wp_yaw + 180) % 360 - 180)
            if yaw_diff > 90.0:
                if wp.get_left_lane() is not None:
                    wp = wp.get_left_lane()
                elif wp.get_right_lane() is not None:
                    wp = wp.get_right_lane()

            step = 2.0
            start_xy = (wp.transform.location.x, -wp.transform.location.y)
            self.waypoints.append(start_xy)

            covered = 0.0
            loop_found = False
            min_loop_distance = 30.0
            loop_threshold = 5.0

            while covered < self.distance:
                next_wps = wp.next(step)
                if not next_wps:
                    break
                wp = next_wps[0]
                pt = (wp.transform.location.x, -wp.transform.location.y)
                self.waypoints.append(pt)
                covered += step
                if covered > min_loop_distance:
                    d = math.hypot(pt[0] - start_xy[0], pt[1] - start_xy[1])
                    if d < loop_threshold:
                        loop_found = True
                        break

            if loop_found:
                # Boucle fermee : on repete le circuit pour couvrir le
                # budget. C'est aussi ce qui donne a la baseline des
                # occasions de fermeture de boucle, sans quoi la
                # comparaison serait deloyale dans l'autre sens.
                repetitions = max(2, int(self.distance / max(covered, 1.0)) + 1)
                self.waypoints = self.waypoints * repetitions
                self.get_logger().info(
                    f"Boucle detectee apres {covered:.1f}m, circuit repete "
                    f"{repetitions} fois -> {len(self.waypoints)} waypoints."
                )
            else:
                self.get_logger().info(
                    f"{len(self.waypoints)} waypoints generes sur "
                    f"{covered:.1f}m (pas de boucle detectee)."
                )

            return True

        except (RuntimeError, IndexError, AttributeError) as e:
            self.get_logger().error(f"Erreur de generation des waypoints : {e}")
            return False

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        position = msg.pose.pose.position
        current_pos = np.array([position.x, position.y])

        if self.last_position is not None:
            pas = float(np.linalg.norm(current_pos - self.last_position))
            if pas < 10.0:   # garde-fou contre un saut aberrant
                self.distance_traveled += pas
        self.last_position = current_pos

        # --- Vitesse ---
        # Lue sur le twist de /odom et non sur
        # /carla/ego/vehicle/vehicle_status : ce topic n'est JAMAIS
        # publie par bridge_node. La version precedente s'y abonnait, si
        # bien que current_velocity restait a 0.0 pendant tout le run --
        # visible dans les journaux sous la forme "vitesse=0.00m/s" alors
        # que la distance parcourue augmentait. Consequence : le plancher
        # de throttle (if speed < 0.5) etait applique en permanence.
        v = msg.twist.twist.linear
        self.current_velocity = math.hypot(v.x, v.y)

        self.distance_pub.publish(
            Float32(data=float(self.distance_traveled)))

    # ------------------------------------------------------------------
    # Perception des obstacles
    # ------------------------------------------------------------------

    def _refresh_obstacles(self):
        """Releve la position des autres vehicules autour de l'ego.

        Interroge CARLA a obstacle_refresh_rate (2 Hz par defaut) et non
        a la cadence de controle : un aller-retour reseau a 10 Hz
        ralentirait la boucle. Les positions sont converties dans le
        repere des waypoints (x, -y).
        """
        if self.world is None or self.ego_actor is None:
            return

        try:
            ego_loc = self.ego_actor.get_transform().location
            releve = []
            for actor in self.world.get_actors().filter('vehicle.*'):
                if actor.id == self.ego_actor.id:
                    continue
                loc = actor.get_transform().location
                if loc.distance(ego_loc) > self.obstacle_scan_radius:
                    continue
                releve.append((loc.x, -loc.y))

            # Les pietons comptent aussi comme obstacles.
            for actor in self.world.get_actors().filter('walker.*'):
                loc = actor.get_transform().location
                if loc.distance(ego_loc) > self.obstacle_scan_radius:
                    continue
                releve.append((loc.x, -loc.y))

            self.obstacles = releve
        except RuntimeError:
            pass

    def _closest_obstacle_ahead(self):
        """Obstacle le plus proche DANS LE COULOIR de circulation.

        Meme methode que vehicle_controller_active_slam : on projette
        chaque obstacle dans le repere du vehicule et on ne retient que
        ceux dont l'ecart lateral est inferieur a la demi-largeur du
        couloir. Un simple filtre angulaire retiendrait les vehicules en
        sens inverse ou sur une voie parallele, et l'ego freinerait en
        permanence en trafic dense.
        """
        if not self.obstacles or self.current_pose is None:
            return None

        vx = self.current_pose.position.x
        vy = self.current_pose.position.y
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        half_width = self.obstacle_corridor_width / 2.0

        closest = None
        for ox, oy in self.obstacles:
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

    # ------------------------------------------------------------------
    # Feux tricolores
    # ------------------------------------------------------------------

    def _red_light_distance(self):
        """Distance a la ligne d'arret d'un feu rouge, ou None.

        get_traffic_light() se declenche jusqu'a une vingtaine de metres
        en amont : on mesure donc la distance reelle a la ligne d'arret
        pour freiner progressivement au lieu de bloquer les roues.
        """
        if not self.respect_traffic_lights or self.ego_actor is None:
            return None
        try:
            tl = self.ego_actor.get_traffic_light()
            if tl is None:
                return None
            if tl.get_state() != carla.TrafficLightState.Red:
                return None

            veh_loc = self.ego_actor.get_transform().location
            stop_wps = tl.get_stop_waypoints()
            if not stop_wps:
                return 0.0

            return min(
                veh_loc.distance(w.transform.location) for w in stop_wps
            )
        except RuntimeError:
            return None

    # ------------------------------------------------------------------
    # Suivi de trajectoire
    # ------------------------------------------------------------------

    def find_closest_waypoint_index(self, global_search=False):
        if not self.waypoints or self.current_pose is None:
            return self.current_wp_index

        pos = np.array([self.current_pose.position.x,
                        self.current_pose.position.y])

        if global_search:
            search_range = range(0, len(self.waypoints))
        else:
            start = max(0, self.current_wp_index - 5)
            end = min(len(self.waypoints), self.current_wp_index + 50)
            search_range = range(start, end)

        best_idx = self.current_wp_index
        best_dist = float('inf')
        for i in search_range:
            wx, wy = self.waypoints[i]
            d = math.hypot(pos[0] - wx, pos[1] - wy)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def find_target_waypoint(self):
        if not self.waypoints or self.current_pose is None:
            return None

        closest_idx = self.find_closest_waypoint_index()
        self.current_wp_index = max(self.current_wp_index, closest_idx)

        if self.current_wp_index >= len(self.waypoints) - 1:
            return None

        px = self.current_pose.position.x
        py = self.current_pose.position.y
        accumulated = 0.0
        idx = self.current_wp_index

        while idx < len(self.waypoints) - 1 and accumulated < self.lookahead_distance:
            wx, wy = self.waypoints[idx]
            accumulated += math.hypot(wx - px, wy - py)
            px, py = wx, wy
            idx += 1

        idx = min(idx, len(self.waypoints) - 1)
        return self.waypoints[idx]

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ------------------------------------------------------------------
    # Commande
    # ------------------------------------------------------------------

    def pure_pursuit_control(self):
        cmd = CarlaEgoVehicleControl()
        cmd.manual_gear_shift = False

        target = self.find_target_waypoint()

        # --- Fin de l'itineraire ---
        if target is None:
            cmd.throttle = 0.0
            cmd.brake = 1.0
            self.finished = True
            self.get_logger().info(
                f"TRAJET TERMINE (fin de l'itineraire) | "
                f"distance parcourue={self.distance_traveled:.1f}m"
            )
            self.cmd_pub.publish(cmd)
            os.killpg(os.getpgid(0), signal.SIGINT)
            # RETURN indispensable : sans lui, l'execution continuait
            # jusqu'a target[0] sur un target valant None. Le signal est
            # asynchrone, il n'interrompt pas forcement a temps.
            return cmd

        self.waiting_deliberately = False

        # --- Feu rouge : freinage progressif ---
        light_distance = self._red_light_distance()
        if light_distance is not None and light_distance < self.light_slow_distance:
            self.waiting_deliberately = True

            if light_distance <= self.light_stop_distance:
                cmd.throttle = 0.0
                cmd.brake = 1.0
                cmd.reverse = False      # jamais la marche arriere pour freiner
                if not self.stopped_at_light:
                    self.stopped_at_light = True
                    self.get_logger().info(
                        f"FEU ROUGE a {light_distance:.1f}m : arret.")
                return cmd

            # Zone d'approche : on ralentit au lieu de bloquer les roues
            # a 0.5 m, ce qui faisait franchir la ligne a 6 m/s.
            ratio = (
                (light_distance - self.light_stop_distance)
                / max(self.light_slow_distance - self.light_stop_distance, 0.1)
            )
            cmd.throttle = float(self.max_throttle * max(0.0, min(1.0, ratio)))
            cmd.brake = 0.0
        else:
            self.stopped_at_light = False

        # --- Panneaux stop ---
        if self.ego_actor is not None and self.stop_signs:
            try:
                veh_loc = self.ego_actor.get_transform().location
                for sign in self.stop_signs:
                    if sign.id in self.stopped_signs_ids:
                        continue
                    if veh_loc.distance(sign.transform.location) >= self.stop_sign_distance:
                        continue

                    if not self.is_stopping_at_sign:
                        self.is_stopping_at_sign = True
                        self.stop_wait_counter = 0
                        self.get_logger().info("PANNEAU STOP : arret.")

                    if self.stop_wait_counter < 30:      # ~3 s a 10 Hz
                        self.stop_wait_counter += 1
                        self.waiting_deliberately = True
                        cmd.throttle = 0.0
                        cmd.brake = 1.0
                        cmd.reverse = False
                        return cmd

                    self.stopped_signs_ids.add(sign.id)
                    self.is_stopping_at_sign = False
                    self.get_logger().info("Arret marque, reprise.")
                    break
            except RuntimeError:
                pass

        # --- Direction ---
        pos_x = self.current_pose.position.x
        pos_y = self.current_pose.position.y
        angle_error = self.normalize_angle(
            math.atan2(target[1] - pos_y, target[0] - pos_x) - self.current_yaw
        )

        steering = max(-self.max_steering_angle,
                       min(self.max_steering_angle, angle_error * 0.7))

        # --- Loi longitudinale, identique au controleur Active SLAM ---
        heading_factor = 1.0 - min(abs(angle_error) / math.radians(100), 1.0)
        throttle = self.max_throttle * heading_factor

        speed = self.current_velocity
        if speed < 0.5:
            throttle = max(throttle, self.min_throttle)

        # --- Obstacles ---
        obstacle_distance = self._closest_obstacle_ahead()

        if obstacle_distance is not None:
            if obstacle_distance <= self.obstacle_stop_distance:
                if self.obstacle_wait_start is None:
                    self.obstacle_wait_start = time.time()
                waited = time.time() - self.obstacle_wait_start

                # Attendre derriere une file arretee a un feu est normal.
                # Au-dela de max_obstacle_wait, l'obstacle est considere
                # comme definitif (vehicule gare, epave) et l'attente
                # cesse d'etre "deliberee" : le chien de garde
                # anti-blocage reprend la main et declenche une manoeuvre
                # de degagement.
                self.waiting_deliberately = waited < self.max_obstacle_wait

                cmd.throttle = 0.0
                cmd.brake = 1.0
                cmd.reverse = False
                if self.command_counter % 20 == 0:
                    self.get_logger().warn(
                        f"OBSTACLE DEVANT a {obstacle_distance:.1f}m : "
                        f"arret, attente {waited:.0f}s/"
                        f"{self.max_obstacle_wait:.0f}s")
                self.command_counter += 1
                return cmd

            self.obstacle_wait_start = None

            ratio = (
                (obstacle_distance - self.obstacle_stop_distance)
                / max(self.obstacle_slow_distance - self.obstacle_stop_distance, 0.1)
            )
            throttle = min(throttle,
                           self.max_throttle * max(0.0, min(1.0, ratio)))
        else:
            self.obstacle_wait_start = None

        # Le plafond du feu rouge s'applique aussi.
        if light_distance is not None and light_distance < self.light_slow_distance:
            throttle = min(throttle, cmd.throttle)

        self.command_counter += 1
        if self.command_counter % 20 == 0:
            self.get_logger().info(
                f"POURSUITE : wp={self.current_wp_index}/{len(self.waypoints) - 1} "
                f"cap_err={math.degrees(angle_error):.0f}deg "
                f"vitesse={speed:.2f}m/s steer={steering:.2f} "
                f"throttle={throttle:.2f} "
                f"distance={self.distance_traveled:.0f}m"
            )

        cmd.throttle = float(max(0.0, min(1.0, throttle)))
        cmd.steer = float(-steering)
        cmd.brake = 0.0
        cmd.reverse = False
        return cmd

    # ------------------------------------------------------------------
    # Anti-blocage
    # ------------------------------------------------------------------

    def check_stuck(self):
        """Detection de blocage sur le deplacement reel.

        Les arrets deliberes -- feu rouge, panneau stop, file derriere un
        obstacle -- ne sont pas des blocages et ne doivent pas declencher
        de manoeuvre de degagement.
        """
        if self.waiting_deliberately or self.current_pose is None:
            self.stuck_counter = 0
            self._stuck_ref_pos = None
            return False

        current_pos = np.array([self.current_pose.position.x,
                                self.current_pose.position.y])

        if self._stuck_ref_pos is None:
            self._stuck_ref_pos = current_pos
            self.stuck_counter = 0
            return False

        if float(np.linalg.norm(current_pos - self._stuck_ref_pos)) > 0.2:
            self._stuck_ref_pos = current_pos
            self.stuck_counter = 0
            return False

        self.stuck_counter += 1

        if self.stuck_counter > self.stuck_threshold:
            self.get_logger().warn(
                f"VEHICULE BLOQUE depuis {self.stuck_counter / 10.0:.0f}s "
                f"hors arret delibere : manoeuvre de degagement."
            )
            self._stuck_ref_pos = current_pos
            self.stuck_counter = 0
            return True

        return False

    def recovery_maneuver(self):
        """Marche arriere braquee, puis recalage global sur l'itineraire.

        L'ancienne version commandait throttle=0, brake=0, steer=0 : elle
        ne degageait rien, le vehicule restait immobile pendant 20 cycles
        puis reprenait exactement dans l'etat qui l'avait bloque.
        """
        cmd = CarlaEgoVehicleControl()
        cmd.manual_gear_shift = False

        if self.recovery_counter < self.recovery_cycles:
            self.recovery_counter += 1
            cmd.throttle = 0.4
            cmd.brake = 0.0
            cmd.reverse = True
            cmd.steer = float(self.recovery_direction)
            return cmd

        self.is_recovering = False
        self.recovery_counter = 0
        self.stuck_counter = 0
        # On alterne le sens de braquage : si la premiere tentative n'a
        # pas degage, la suivante essaie l'autre cote.
        self.recovery_direction = -self.recovery_direction
        self.current_wp_index = self.find_closest_waypoint_index(
            global_search=True)
        self.get_logger().warn(
            f"Recalage sur le waypoint {self.current_wp_index} "
            f"apres degagement."
        )
        return self.pure_pursuit_control()

    # ------------------------------------------------------------------
    # Boucle principale
    # ------------------------------------------------------------------

    def control_loop(self):
        if self.current_pose is None or not self.waypoints or self.finished:
            return

        # --- Budget de distance ---
        if self.distance_traveled >= self.distance:
            cmd = CarlaEgoVehicleControl()
            cmd.throttle = 0.0
            cmd.brake = 1.0
            self.cmd_pub.publish(cmd)
            self.finished = True

            self.get_logger().info(
                f"FIN DE RUN (budget_reached) : budget de distance atteint "
                f"({self.distance_traveled:.0f}m / {self.distance:.0f}m) | "
                f"strategie=predefinie"
            )

            os.killpg(os.getpgid(0), signal.SIGINT)
            return

        # --- Anti-blocage, desormais reellement branche ---
        if self.is_recovering:
            cmd = self.recovery_maneuver()
            self.cmd_pub.publish(cmd)
            return

        if self.check_stuck():
            self.is_recovering = True
            self.recovery_counter = 0
            cmd = self.recovery_maneuver()
            self.cmd_pub.publish(cmd)
            return

        self.cmd_pub.publish(self.pure_pursuit_control())


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerEval()
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
