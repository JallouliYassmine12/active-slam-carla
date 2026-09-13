#!/usr/bin/env python3
"""
Noeud de decision Active SLAM : selectionne automatiquement la prochaine
destination du vehicule en ponderant 5 criteres, comme decrit dans le
sujet de stage :

- gain d'information attendu       (grille d'occupation RTAB-Map)
- potentiel de fermeture de boucle (graphe RTAB-Map)
- cout de distance jusqu'au candidat
- securite (proximite d'obstacles / pietons / vehicules)
- incertitude de localisation      (pose RTAB-Map)

Les 4 premiers criteres sont evalues PAR CANDIDAT. Le cinquieme,
l'incertitude de localisation, est un etat GLOBAL du systeme (il ne
depend pas du candidat evalue) : il est donc modelise comme un facteur
qui redistribue dynamiquement le poids entre gain d'information et
fermeture de boucle (voir strategies.select_weighted). C'est la partie
"consciente de l'incertitude" du sujet : quand la localisation se
degrade, le vehicule privilegie les destinations permettant de se
recaler plutot que d'explorer du terrain inconnu.

Deux mecanismes d'arret distincts (voir stop_reason) :
- 'exploration_complete'  : plus aucun candidat n'offre de gain
                            d'information ni de fermeture de boucle
                            significatifs -> carte jugee terminee
- 'critical_uncertainty'  : filet de securite, la localisation a
                            diverge au point que les autres criteres
                            ne sont plus fiables

Entrees :
/active_slam/candidate_goals  (geometry_msgs/msg/PoseArray)  <- candidate_generator
/grid_prob_map                (nav_msgs/msg/OccupancyGrid)
/localization_pose            (geometry_msgs/msg/PoseWithCovarianceStamped)
/mapGraph                     (rtabmap_msgs/msg/MapGraph)
/vehicle/arrived              (std_msgs/msg/Bool)            <- vehicle_controller_active_slam
<obstacles_topic> (parametre) (geometry_msgs/msg/PoseArray)  - voir safety_evaluator.py

Sorties :
/active_slam/goal                 (geometry_msgs/msg/PoseStamped) -> vehicle_controller_active_slam
/active_slam/exploration_complete (std_msgs/msg/Bool)

Un fichier CSV optionnel (parametre `log_file`) enregistre chaque
decision avec le detail des scores, pour l'analyse comparative finale
(weighted vs random vs distance vs info_gain).
"""

import math
import csv
import os
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32, String
from rtabmap_msgs.msg import MapGraph

from active_slam_decision.information_gain import (
    compute_information_gain,
    compute_path_information_gain,
)
from active_slam_decision.localization_uncertainty import (
    compute_localization_uncertainty,
    normalize_uncertainty,
)
from active_slam_decision.loop_closure_potential import (
    compute_loop_closure_potential,
)
from active_slam_decision.safety_evaluator import compute_safety_score
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from active_slam_decision import strategies


@dataclass
class Candidate:
    id: int
    x: float
    y: float
    # Distance ROUTIERE transmise par candidate_generator dans
    # position.z. Vaut 0.0 si le message vient d'une source qui ne la
    # fournit pas : le cout de distance retombe alors sur la distance a
    # vol d'oiseau, sans regression silencieuse.
    road_distance: float = 0.0


class DecisionMaker(Node):

    def __init__(self):
        super().__init__('active_slam_decision_maker')

        # --- Parametres ---
        self.declare_parameter('strategy', 'weighted')  # weighted | random | distance | info_gain
        self.declare_parameter('weight_information_gain', 0.30)
        self.declare_parameter('weight_localization_uncertainty', 0.20)
        self.declare_parameter('weight_loop_closure', 0.20)
        self.declare_parameter('weight_distance', 0.15)
        self.declare_parameter('weight_safety', 0.15)
        self.declare_parameter('sensor_range', 20.0)

        # --- Gain d'information evalue le long du trajet ---
        # Pas d'echantillonnage du segment vehicule -> candidat, aligne
        # sur le pas du parcours routier (waypoint_spacing et route_step
        # valent tous deux 8 m).
        self.declare_parameter('path_sample_step', 8.0)
        # Rayon mesure autour de chaque point echantillonne : environ
        # une voie et demie. Assez large pour ne pas dependre d'une
        # cellule isolee, assez etroit pour que deux rues voisines ne se
        # confondent pas -- et pour que le calcul reste rapide, le
        # disque parcouru croissant comme le carre du rayon.
        self.declare_parameter('path_corridor_radius', 6.0)
        self.declare_parameter('max_candidate_distance', 150.0)
        self.declare_parameter(
            'obstacles_topic',
            '/obstacle_detection/obstacles'
        )
        self.declare_parameter('log_file', '')

        # Liste tabou : evite de re-choisir un candidat trop proche d'une
        # destination recemment visitee (evite les allers-retours en boucle
        # une fois que le gain d'information local tombe a 0).
        self.declare_parameter('tabu_size', 5)
        self.declare_parameter('tabu_radius', 10.0)
        self.declare_parameter('unreachable_exclusion_radius', 6.0)

        # Detection de stagnation : arrete la selection automatique si,
        # pendant plusieurs decisions consecutives, aucun candidat n'offre
        # ni gain d'information ni potentiel de fermeture de boucle
        # significatif (= exploration consideree terminee).
        self.declare_parameter('stagnation_info_threshold', 0.05)
        self.declare_parameter('stagnation_loop_threshold', 0.05)
        # NOTE : ce seuil n'entre plus dans la condition de stagnation.
        # L'incertitude croissant de facon monotone (elle ne redescend
        # jamais sans fermeture de boucle), l'exiger faible rendait
        # l'arret normal structurellement inatteignable. Conserve pour
        # compatibilite des fichiers de launch.
        self.declare_parameter('stagnation_uncertainty_threshold', 0.20)
        self.declare_parameter('stagnation_patience', 5)

        # Reference de normalisation de l'incertitude : alignee sur le
        # seuil critique, pour que l'incertitude normalisee couvre
        # exactement 0->1 sur la plage de fonctionnement admise.
        # (Valable depuis le passage a une normalisation logarithmique,
        # cf. localization_uncertainty.normalize_uncertainty.)
        self.declare_parameter('uncertainty_reference', 50.0)
        self.declare_parameter('uncertainty_critical_threshold', 50.0)

        # --- Budget de run (comparabilite de la campagne) ---
        # Sans budget, un run ne se termine que sur stagnation (jamais
        # atteinte sur une carte urbaine, ou le generateur publie en
        # permanence 15 a 45 candidats atteignables) ou sur l'arret
        # securite (qui depend de la derive, donc de la trajectoire
        # suivie, donc de la strategie testee). Les cinq strategies
        # s'arreteraient alors a des instants differents et leurs
        # metriques ne seraient pas comparables : le temps d'exploration
        # deviendrait une variable non controlee.
        # Un budget fixe et identique pour toutes les strategies leve
        # cette objection. 0 = desactive.
        self.declare_parameter('max_run_duration', 0.0)     # secondes
        self.declare_parameter('max_destinations', 0)       # nombre de buts atteints

        # Budget en distance parcourue. C'est la seule unite commune au
        # SLAM classique (trajectoire predefinie) et a l'Active SLAM : la
        # duree depend de la chance aux feux rouges, et le nombre de
        # destinations n'a aucun equivalent sur une trajectoire fixe.
        self.declare_parameter('max_distance', 0.0)         # metres

        self.strategy = self.get_parameter('strategy').value

        self.weights = {
            'information_gain':
                self.get_parameter('weight_information_gain').value,
            'localization_uncertainty':
                self.get_parameter('weight_localization_uncertainty').value,
            'loop_closure':
                self.get_parameter('weight_loop_closure').value,
            'distance':
                self.get_parameter('weight_distance').value,
            'safety':
                self.get_parameter('weight_safety').value,
        }

        self.sensor_range = self.get_parameter('sensor_range').value
        self.path_sample_step = float(
            self.get_parameter('path_sample_step').value
        )
        self.path_corridor_radius = float(
            self.get_parameter('path_corridor_radius').value
        )
        self.max_candidate_distance = self.get_parameter(
            'max_candidate_distance'
        ).value

        obstacles_topic = self.get_parameter('obstacles_topic').value
        log_file = self.get_parameter('log_file').value

        self.tabu_size = self.get_parameter('tabu_size').value
        self.unreachable_exclusion_radius = self.get_parameter(
            'unreachable_exclusion_radius'
        ).value
        self.tabu_radius = self.get_parameter(
            'tabu_radius'
        ).value

        self.stagnation_info_threshold = self.get_parameter(
            'stagnation_info_threshold'
        ).value

        self.stagnation_loop_threshold = self.get_parameter(
            'stagnation_loop_threshold'
        ).value

        self.stagnation_uncertainty_threshold = self.get_parameter(
            'stagnation_uncertainty_threshold'
        ).value

        self.stagnation_patience = self.get_parameter(
            'stagnation_patience'
        ).value

        self.uncertainty_reference = self.get_parameter(
            'uncertainty_reference'
        ).value

        self.uncertainty_critical_threshold = self.get_parameter(
            'uncertainty_critical_threshold'
        ).value

        self.max_run_duration = float(
            self.get_parameter('max_run_duration').value
        )

        self.max_destinations = int(
            self.get_parameter('max_destinations').value
        )


        self.max_distance = float(
            self.get_parameter('max_distance').value
        )

        # --- Etat courant ---
        # --- Grille d'occupation ---
        # Deux sources possibles. /grid_prob_map est un topic OPTIONNEL
        # de RTAB-Map : sa grille probabiliste n'est remplie que dans
        # certaines configurations. Mesure sur valid_raytracing2.log :
        # 100,0 % de cellules a -1, boite englobante qui grandit avec la
        # trajectoire mais aucun contenu. /map porte la grille
        # d'occupation standard.
        #
        # On s'abonne aux deux et on retient celle qui a du contenu,
        # plutot que de figer un nom de topic qu'il faudrait revalider a
        # chaque changement de configuration.
        self.current_grid = None       # celle effectivement utilisee
        self.grille_prob = None        # /grid_prob_map
        self.grille_std = None         # /map
        self.grille_nom = None         # topic retenu, pour le journal
        self.grille_dernier_choix = 0.0
        self.current_pose_cov = None
        self.current_map_graph = None
        self.current_node_id = None
        self.current_obstacles = []

        self.current_x = 0.0
        self.current_y = 0.0

        self.candidates = []
        self.waiting_for_candidates = True

        self.recent_goals = deque(maxlen=self.tabu_size)
        # Contrairement a recent_goals (fenetre glissante de tabu_size
        # buts, pour eviter les allers-retours), cette liste n'oublie
        # jamais : un point signale une fois comme physiquement
        # inatteignable (aucun itineraire routier) le reste pour tout le
        # run, sinon la liste tabou finit par l'oublier et
        # select_weighted le re-choisit indefiniment s'il obtient le
        # meilleur score.
        self.unreachable_points = []

        self.stagnation_count = 0
        self.exploration_done = False
        self.stop_reason = None

        # --- Suivi du budget de run ---
        self.run_start_time = self.get_clock().now()
        self.destinations_reached = 0
        self.last_goal_time = None
        self.consecutive_rejections = 0
        # Instant du premier rejet d'une serie ininterrompue. Sert au
        # critere d'arret : c'est une DUREE sans progres qui doit
        # terminer le run, pas un nombre de messages echanges.
        self.premier_rejet_time = None
        # Instant du dernier repli sur la liste complete de candidats,
        # quand le filtre des points inatteignables les a tous exclus.
        self.dernier_repli_time = None
        self.distance_traveled = 0.0

        # --- Abonnements ---
        self.create_subscription(
            PoseArray,
            '/active_slam/candidate_goals',
            self.on_candidates,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/active_slam/goal_rejected',
            self.on_goal_rejected,
            10
        )


        self.create_subscription(
            OccupancyGrid,
            '/grid_prob_map',
            self.on_grid,
            10
        )

        # Grille d'occupation standard de RTAB-Map. Voir _choisir_grille.
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.on_grid_std,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self.on_pose,
            10
        )

        map_graph_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            MapGraph,
            '/mapGraph',
            self.on_map_graph,
            map_graph_qos
        )

        self.create_subscription(
            PoseArray,
            obstacles_topic,
            self.on_obstacles,
            10
        )

        self.create_subscription(
            Bool,
            '/vehicle/arrived',
            self.on_arrived,
            10
        )

        self.create_subscription(
            Float32,
            '/vehicle/distance_traveled',
            self.on_distance,
            10
        )

        # Evenements de securite publies par le bridge. Compteurs cumules
        # sur la duree du run, reportes dans le CSV a chaque decision.
        self.collisions = {'vehicule': 0, 'pieton': 0, 'obstacle': 0}
        self.create_subscription(
            String,
            '/carla/ego/collision',
            self.on_collision,
            10
        )

        # --- Publication ---
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/active_slam/goal',
            10
        )

        self.exploration_complete_pub = self.create_publisher(
            Bool,
            '/active_slam/exploration_complete',
            10
        )

        # --- Log CSV des decisions ---
        self.log_file = log_file
        self._log_fh = None
        self._log_writer = None

        if self.log_file:
            os.makedirs(
                os.path.dirname(self.log_file),
                exist_ok=True
            )

            write_header = not os.path.exists(self.log_file)

            self._log_fh = open(
                self.log_file,
                'a',
                newline=''
            )

            self._log_writer = csv.writer(self._log_fh)

            if write_header:
                self._log_writer.writerow([
                    'stamp',
                    'strategy',
                    'chosen_x',
                    'chosen_y',
                    'information_gain',
                    'loop_closure',
                    'localization_uncertainty',
                    'safety',
                    'distance',
                    'collisions_vehicules',
                    'collisions_pietons',
                    'collisions_obstacles',
                    # Activite du critere de securite sur l'ENSEMBLE des
                    # candidats de la decision, et non sur le seul candidat
                    # retenu. Sans ces colonnes, un critere qui ecarte des
                    # candidats dangereux a chaque decision est invisible
                    # dans le CSV : on n'y lit que le score du candidat
                    # choisi, qui est justement l'un des plus surs.
                    'nb_candidats',
                    'safety_min',
                    'candidats_penalises',
                    # Etendue des criteres exploratoires sur TOUS
                    # les candidats. Sans elles, on ne lit que le
                    # score du candidat retenu -- or la selection
                    # maximise ce score, donc le gagnant a
                    # mecaniquement le gain le plus eleve, que le
                    # critere discrimine ou qu'il soit sature.
                    'info_gain_min',
                    'info_gain_max',
                    'loop_closure_max'
                ])

        # Surveillance du budget en duree. Un timer est necessaire : le
        # budget doit pouvoir expirer meme si le vehicule est immobilise
        # (feu rouge, file d'attente derriere un NPC), donc en dehors de
        # tout evenement 'arrived'.
        if self.max_run_duration > 0.0:
            self.create_timer(1.0, self._check_time_budget)

        self.get_logger().info(
            f"active_slam_decision_maker demarre "
            f"(strategie='{self.strategy}') | budget : "
            f"duree={'illimitee' if self.max_run_duration <= 0.0 else f'{self.max_run_duration:.0f}s'}, "
            f"destinations={'illimitees' if self.max_destinations <= 0 else self.max_destinations}, "
            f"distance={'illimitee' if self.max_distance <= 0.0 else f'{self.max_distance:.0f}m'}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_grid(self, msg):
        self.grille_prob = msg
        self._choisir_grille()

    def on_grid_std(self, msg):
        self.grille_std = msg
        self._choisir_grille()

    @staticmethod
    def _proportion_connue(grille):
        """Fraction de cellules NON inconnues, entre 0 et 1.

        count() s'execute en C : le cout reste negligeable meme sur une
        grille de plusieurs millions de cellules.
        """
        if grille is None or grille.info.width == 0:
            return -1.0
        donnees = grille.data
        total = len(donnees)
        if total == 0:
            return -1.0
        return 1.0 - donnees.count(-1) / float(total)

    def _choisir_grille(self):
        """Retient la grille qui contient reellement des cellules connues.

        /grid_prob_map est un topic optionnel de RTAB-Map : dans cette
        configuration il publiait une carte integralement a -1 (mesure :
        100,0 % d'inconnu, boite englobante grandissant avec la
        trajectoire). Le critere de gain d'information mesurait donc la
        proportion d'inconnu dans une grille ou tout est inconnu, et
        saturait a 1,000 quels que soient les reglages.

        Choisir automatiquement evite de figer un nom de topic qu'il
        faudrait revalider a chaque changement de configuration.
        """
        maintenant = self.get_clock().now().nanoseconds / 1e9
        if maintenant - self.grille_dernier_choix < 2.0:
            # Le choix ne peut pas changer plusieurs fois par seconde ;
            # on met simplement a jour la grille deja retenue.
            if self.grille_nom == '/map':
                self.current_grid = self.grille_std
            elif self.grille_nom == '/grid_prob_map':
                self.current_grid = self.grille_prob
            else:
                self.current_grid = self.grille_std or self.grille_prob
            return

        self.grille_dernier_choix = maintenant

        connu_prob = self._proportion_connue(self.grille_prob)
        connu_std = self._proportion_connue(self.grille_std)

        if connu_std >= connu_prob:
            retenue, nom = self.grille_std, '/map'
            ecartee, nom_ecartee, connu_ecartee = (
                self.grille_prob, '/grid_prob_map', connu_prob
            )
            connu_retenu = connu_std
        else:
            retenue, nom = self.grille_prob, '/grid_prob_map'
            ecartee, nom_ecartee, connu_ecartee = (
                self.grille_std, '/map', connu_std
            )
            connu_retenu = connu_prob

        self.current_grid = retenue

        if nom != self.grille_nom and retenue is not None:
            self.grille_nom = nom
            self.get_logger().warn(
                f"GRILLE ACTIVE : {nom} "
                f"(connu={100.0 * connu_retenu:.1f}%) -- "
                f"{nom_ecartee} "
                f"{'ecartee' if ecartee is not None else 'absente'} "
                f"(connu={100.0 * max(connu_ecartee, 0.0):.1f}%)"
            )

    def on_pose(self, msg):
        self.current_pose_cov = msg
        self.current_x = msg.pose.pose.position.x
        # --- Convention de repere : inversion de Y ---
        # Le nuage LiDAR est publie dans la convention MAIN
        # GAUCHE de CARLA. Le repere que RTAB-Map en deduit est
        # donc le miroir lateral du repere 'map' de
        # candidate_generator et obstacle_detector, qui
        # appliquent -(y - origin_y). localization_evaluator
        # compense deja ce miroir avant de calculer l'ATE
        # (est_l['y'] = -est_l['y']) ; la compensation manquait
        # ici, si bien que les criteres geometriques comparaient
        # deux reperes differents.
        #
        # Effet mesure avant correction : la distance annoncee
        # vers le candidat retenu depassait la longueur de
        # l'itineraire routier reel dans 26 decisions sur 29, ce
        # qui est geometriquement impossible. L'erreur valait le
        # double de l'ecart lateral du vehicule a son axe de
        # depart : nulle au demarrage, croissante ensuite,
        # retombant a chaque retour vers cet axe.
        self.current_y = -msg.pose.pose.position.y

        raw_unc = compute_localization_uncertainty(msg)
        norm_unc = normalize_uncertainty(
            raw_unc,
            reference=self.uncertainty_reference
        )

        self.get_logger().info(
            f"DEBUG pose : x={self.current_x:.2f} y={self.current_y:.2f} "
            f"incertitude_brute={raw_unc:.4f}"
            f" incertitude_normalisee={norm_unc:.4f}"
        )

        # Filet de securite : ce n'est PAS la fin normale de l'exploration.
        # Au-dela de ce seuil, la pose estimee a tellement derive que les
        # 4 autres criteres (calcules dans ce repere) ne veulent plus rien
        # dire : on arrete plutot que de produire une carte corrompue.
        if not self.exploration_done and raw_unc > self.uncertainty_critical_threshold:
            self.get_logger().warn(
                f"ARRET SECURITE : incertitude de localisation critique "
                f"({raw_unc:.2f} > seuil {self.uncertainty_critical_threshold}). "
                f"Localisation jugee non fiable, arret pour eviter de continuer "
                f"sur des donnees corrompues."
            )
            self._finish_run(
                'critical_uncertainty',
                f"incertitude brute={raw_unc:.2f} > "
                f"{self.uncertainty_critical_threshold:.0f}"
            )

    def on_map_graph(self, msg):
        self.current_map_graph = msg

        if len(msg.poses_id) > 0:
            self.current_node_id = msg.poses_id[-1]

            self.get_logger().info(
                f"DEBUG mapGraph: {len(msg.poses_id)} noeuds, "
                f"current_node_id={self.current_node_id}"
            )

    def on_obstacles(self, msg):
        self.current_obstacles = [
            (p.position.x, p.position.y)
            for p in msg.poses
        ]

    def on_candidates(self, msg):
        self.candidates = [
            Candidate(
                id=i,
                x=p.position.x,
                y=p.position.y,
                road_distance=p.position.z
            )
            for i, p in enumerate(msg.poses)
        ]

        self.get_logger().info(
            f"CANDIDATS RECUS : {len(self.candidates)} | "
            f"pose=({self.current_x:.2f}, {self.current_y:.2f}) | "
            f"grid={'OK' if self.current_grid is not None else 'NONE'} | "
            f"pose_cov={'OK' if self.current_pose_cov is not None else 'NONE'} | "
            f"graph={'OK' if self.current_map_graph is not None else 'NONE'}"
        )

        if self.waiting_for_candidates:
            self.waiting_for_candidates = False
            self.choose_next_goal()

    def on_arrived(self, msg):
        if not msg.data:
            return

        # --- Filtrage des buts rejetes par le planificateur routier ---
        # vehicle_controller_active_slam republie 'arrived' lorsqu'il ne
        # trouve aucun itineraire vers le but ("BUT REJETE"), afin d'en
        # redemander un autre. Sans filtrage ces rejets sont comptes
        # comme des destinations atteintes : sur le run de controle le
        # compteur affichait 15 alors que le vehicule n'en avait
        # physiquement atteint que 13. Le budget cesse alors d'etre
        # identique d'une strategie a l'autre, ce qui invalide toute
        # comparaison.
        # Un rejet se distingue sans ambiguite d'une arrivee reelle par
        # son instant : il survient quelques millisecondes apres la
        # publication du but, alors que la destination la plus proche
        # est a 25 m, soit plus de 1.6 s de trajet meme a la vitesse
        # maximale observee (15 m/s).
        if self.last_goal_time is not None:
            delai = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            if delai < 1.0:
                self.consecutive_rejections += 1
                self.get_logger().warn(
                    f"BUT NON ROUTABLE (rejete {delai * 1000:.0f} ms apres publication) : "
                    f"non compte comme destination atteinte "
                    f"(rejets consecutifs={self.consecutive_rejections})"
                )
                if self.consecutive_rejections >= 20:
                    self._finish_run(
                        'no_reachable_goal',
                        f"20 buts consecutifs rejetes par le planificateur routier"
                    )
                    return
                self.choose_next_goal()
                return

        self.consecutive_rejections = 0
        self.premier_rejet_time = None
        self.destinations_reached += 1

        if (
            self.max_destinations > 0
            and self.destinations_reached >= self.max_destinations
        ):
            self._finish_run(
                'budget_reached',
                f"budget de destinations atteint "
                f"({self.destinations_reached}/{self.max_destinations})"
            )
            return

        self.get_logger().info(
            "Vehicule arrive : selection de la prochaine destination"
        )
        self.choose_next_goal()

    def on_distance(self, msg):
        self.distance_traveled = float(msg.data)

        if self.max_distance <= 0.0 or self.exploration_done:
            return

        if self.distance_traveled >= self.max_distance:
            self._finish_run(
                'budget_reached',
                f"budget de distance atteint "
                f"({self.distance_traveled:.0f}m / {self.max_distance:.0f}m)"
            )

    def _elapsed_seconds(self):
        delta = self.get_clock().now() - self.run_start_time
        return delta.nanoseconds / 1e9

    def _check_time_budget(self):
        if self.exploration_done:
            return

        elapsed = self._elapsed_seconds()

        if elapsed >= self.max_run_duration:
            self._finish_run(
                'budget_reached',
                f"budget de duree atteint "
                f"({elapsed:.1f}s / {self.max_run_duration:.0f}s)"
            )

    def _finish_run(self, reason, detail):
        """
        Fin de run commune aux differents motifs d'arret. Publie
        exploration_complete puis ferme le noeud apres 2 s, ce qui
        declenche l'arret de tout le launch via l'OnProcessExit.
        """
        if self.exploration_done:
            return

        self.get_logger().info(
            f"FIN DE RUN ({reason}) : {detail} | "
            f"destinations atteintes={self.destinations_reached}, "
            f"distance parcourue={self.distance_traveled:.0f}m, "
            f"duree={self._elapsed_seconds():.1f}s"
        )

        self.exploration_done = True
        self.stop_reason = reason
        self.exploration_complete_pub.publish(Bool(data=True))
        self.create_timer(2.0, self._shutdown_after_exploration)

    def on_goal_rejected(self, msg):
        self.unreachable_points.append(
            (msg.pose.position.x, msg.pose.position.y)
        )

        # --- Relance de la decision ---
        # Ce rappel se contentait d'enregistrer le point : il ne
        # redemandait AUCUNE nouvelle destination. choose_next_goal()
        # n'etant appele que depuis on_arrived() et depuis la toute
        # premiere reception de candidats, un but rejete figeait le
        # vehicule jusqu'a l'expiration du budget de duree :
        # current_goal restait a None et le controleur freinait.
        #
        # Observe sur un run Town01_Opt : "BUT REJETE (aucun
        # itineraire routier)" a t+2 s, puis plus une seule decision
        # de tout le run, pose figee a (0.3, -0.05).
        #
        # Le compteur de destinations n'est PAS incremente : un but
        # ecarte n'est pas une destination atteinte. C'est ce qui
        # distingue ce chemin de on_arrived(), et ce qui garantit que
        # le budget reste identique d'une strategie a l'autre.
        if self.exploration_done:
            return

        self.consecutive_rejections += 1
        self.get_logger().warn(
            f"BUT ECARTE : ({msg.pose.position.x:.1f}, "
            f"{msg.pose.position.y:.1f}) marque inatteignable "
            f"({len(self.unreachable_points)} au total, "
            f"{self.consecutive_rejections} rejets consecutifs). "
            f"Nouvelle destination demandee."
        )

        # Garde-fou : si plus aucun candidat n'est routable, mieux
        # vaut terminer le run proprement que boucler jusqu'au budget.
        #
        # Le critere est une DUREE, pas un nombre de rejets. Le seuil
        # precedent -- 20 rejets consecutifs -- terminait le run apres
        # 180 millisecondes, parce que ces echanges se font par
        # messages ROS et non a la vitesse du vehicule. Un critere
        # d'arret doit mesurer un temps pendant lequel le systeme n'a
        # pas progresse.
        if self.premier_rejet_time is None:
            self.premier_rejet_time = self.get_clock().now()

        sans_progres = (
            self.get_clock().now() - self.premier_rejet_time
        ).nanoseconds / 1e9

        if sans_progres > 60.0:
            self._finish_run(
                'no_reachable_goal',
                f"aucune destination acceptee pendant "
                f"{sans_progres:.0f}s "
                f"({self.consecutive_rejections} buts ecartes)"
            )
            return

        self.choose_next_goal()

    # ------------------------------------------------------------------
    # Logique de decision
    # ------------------------------------------------------------------

    def choose_next_goal(self):
        self.get_logger().info(
            f"CHOOSE_NEXT_GOAL : {len(self.candidates)} candidats"
        )

        if self.exploration_done:
            # Deja arrete : on ignore les nouveaux "arrived"/candidats
            # plutot que de repartir en boucle.
            return

        if not self.candidates:
            self.get_logger().warn(
                "Aucun candidat disponible pour le moment"
            )
            return

        reachable = [
            c
            for c in self.candidates
            if math.hypot(
                c.x - self.current_x,
                c.y - self.current_y
            ) <= self.max_candidate_distance
        ]

        if not reachable:
            # --- Repli BORNE ---
            # Ce repli rendait max_candidate_distance inoperant :
            # quand le generateur ne publie que des candidats
            # lointains, on repartait sur la liste COMPLETE et le
            # plafond de 80 m ne s'appliquait plus. Mesure sur
            # valid_retour25.log : buts retenus a 123.9, 136.9 et
            # 154.2 m, tous au-dela du plafond.
            #
            # Le repli garde sa raison d'etre -- ne jamais figer le
            # vehicule faute de candidat exploitable -- mais il ne
            # peut plus ramener les buts les plus eloignes de tous,
            # ceux-la memes que le plafond ecartait.
            reachable = sorted(
                self.candidates,
                key=lambda c: math.hypot(
                    c.x - self.current_x,
                    c.y - self.current_y
                )
            )[:5]

        # --- Filtrage des points connus inatteignables ---
        # Exclut definitivement les candidats trop proches d'un but deja
        # signale comme physiquement inatteignable par vehicle_controller
        # (aucun itineraire routier trouve). Sans ce filtre, un point
        # attractif mais inatteignable ressort de la liste tabou (fenetre
        # glissante) au bout de quelques cycles et se fait re-choisir
        # indefiniment.
        topologically_reachable = [
            c
            for c in reachable
            if all(
                math.hypot(c.x - ux, c.y - uy) > self.unreachable_exclusion_radius
                for ux, uy in self.unreachable_points
            )
        ]

        if topologically_reachable:
            reachable = topologically_reachable
        else:
            # Tous les candidats sont a moins de
            # unreachable_exclusion_radius d'un point deja declare
            # inatteignable.
            #
            # Deux mauvaises reponses ont ete essayees ici :
            #
            # 1. Retomber silencieusement sur la liste complete. Cela
            #    rechoisit un point deja rejete, et la boucle
            #    rejet -> decision -> rejet tourne a la vitesse des
            #    messages ROS : 20 rejets en 180 ms, mesures.
            #
            # 2. Attendre la publication de candidats suivante. Mais
            #    cette liste est calculee depuis la position du
            #    vehicule : tant qu'il ne bouge pas, elle est
            #    identique. L'attente n'a pas d'issue -- vehicule
            #    immobile pendant plus de 3 minutes, mesure.
            #
            # La bonne reponse est le repli CADENCE : on repart sur la
            # liste complete, mais au plus une fois toutes les
            # repli_cooldown secondes. La rafale devient impossible, le
            # vehicule continue d'essayer, et le chien de garde des
            # 60 s sans destination acceptee peut terminer le run
            # proprement si la situation ne se debloque pas.
            repli_cooldown = 5.0
            maintenant = self.get_clock().now()
            depuis_dernier = (
                float('inf') if self.dernier_repli_time is None
                else (maintenant - self.dernier_repli_time).nanoseconds / 1e9
            )

            if depuis_dernier < repli_cooldown:
                self.waiting_for_candidates = True
                return

            self.dernier_repli_time = maintenant
            self.get_logger().warn(
                f"AUCUN CANDIDAT EXPLOITABLE : les "
                f"{len(reachable)} candidats sont tous a moins de "
                f"{self.unreachable_exclusion_radius:.0f} m d'un point "
                f"deja declare inatteignable "
                f"({len(self.unreachable_points)} points). Repli sur la "
                f"liste complete pour ne pas figer le vehicule."
            )

        # --- Filtrage tabou ---
        # Ecarte les candidats trop proches d'une destination recemment
        # visitee, pour eviter que weighted/info_gain ne re-choisissent en
        # boucle les 2-3 memes points une fois le gain d'information local
        # epuise (ce qui se traduit par des allers-retours sans fin). Si le
        # filtre exclut tous les candidats (zone tres restreinte), on
        # retombe sur la liste complete plutot que de bloquer la decision.
        non_tabu = [
            c
            for c in reachable
            if all(
                math.hypot(c.x - gx, c.y - gy) > self.tabu_radius
                for gx, gy in self.recent_goals
            )
        ]

        candidates_pool = non_tabu if non_tabu else reachable

        scores = self._evaluate_candidates(candidates_pool)

        self.get_logger().info(
            f"SCORES CALCULES : {len(scores)} candidats | "
            f"pool={len(candidates_pool)}"
        )

        # --- Detection de stagnation (arret normal) ---
        # Exploration terminee si, pendant plusieurs decisions
        # consecutives, aucun candidat n'offre :
        #   1. de gain d'information significatif
        #   2. ni de potentiel de fermeture de boucle significatif
        # L'incertitude n'entre pas dans ce critere : elle determine
        # OU aller (via la ponderation adaptative), pas QUAND s'arreter.

        best_info_gain = max(
            (
                scores[c.id]['information_gain']
                for c in candidates_pool
            ),
            default=0.0
        )

        best_loop_closure = max(
            (
                scores[c.id]['loop_closure']
                for c in candidates_pool
            ),
            default=0.0
        )

        # Incertitude courante : sert a la ponderation adaptative et au log.
        raw_uncertainty = compute_localization_uncertainty(
            self.current_pose_cov
        )

        norm_uncertainty = normalize_uncertainty(
            raw_uncertainty,
            reference=self.uncertainty_reference
        )

        # Garde-fou : ne pas compter de stagnation tant que RTAB-Map n'a
        # pas encore produit de grille/graphe. Sans cette garde, un
        # info_gain/loop_closure a 0.0 en tout debut de run (faute de
        # donnees, pas faute d'interet) declenchait de fausses stagnations.
        data_ready = (
            self.current_grid is not None
            and self.current_node_id is not None
        )

        stagnation_detected = (
            data_ready
            and best_info_gain < self.stagnation_info_threshold
            and best_loop_closure < self.stagnation_loop_threshold
        )

        if stagnation_detected:
            self.stagnation_count += 1

            self.get_logger().info(
                f"STAGNATION : "
                f"info_gain={best_info_gain:.3f} "
                f"(seuil={self.stagnation_info_threshold:.3f}), "
                f"loop_closure={best_loop_closure:.3f} "
                f"(seuil={self.stagnation_loop_threshold:.3f}) "
                f"[u={norm_uncertainty:.3f}, informatif] "
                f"-> compteur={self.stagnation_count}/"
                f"{self.stagnation_patience}"
            )

        else:
            if self.stagnation_count > 0:
                self.get_logger().info(
                    "STAGNATION INTERROMPUE : "
                    "une condition n'est plus satisfaite -> "
                    "compteur remis a 0"
                )

            self.stagnation_count = 0

        if self.stagnation_count >= self.stagnation_patience:
            self.get_logger().info(
                f"Exploration consideree terminee : "
                f"{self.stagnation_patience} decisions consecutives avec "
                f"gain d'information faible et potentiel de fermeture de "
                f"boucle faible."
            )
            # _finish_run laisse 2 secondes pour que exploration_complete
            # soit bien transmis avant de fermer le noeud.
            self._finish_run(
                'exploration_complete',
                f"{self.stagnation_patience} decisions consecutives sans "
                f"gain d'information ni potentiel de fermeture de boucle"
            )
            return

        if self.strategy == 'random':
            chosen = strategies.select_random(candidates_pool)

        elif self.strategy == 'distance':
            chosen = strategies.select_by_distance(
                candidates_pool,
                scores
            )

        elif self.strategy == 'info_gain':
            chosen = strategies.select_by_information_gain(
                candidates_pool,
                scores
            )

        else:
            chosen = strategies.select_weighted(
                candidates_pool,
                scores,
                self.weights,
                uncertainty=norm_uncertainty
            )

        self.get_logger().info(
            f"CANDIDAT CHOISI : id={chosen.id}, "
            f"x={chosen.x:.2f}, y={chosen.y:.2f} | "
            f"u={norm_uncertainty:.3f} | scores={scores[chosen.id]}"
        )

        self.recent_goals.append(
            (chosen.x, chosen.y)
        )

        self._publish_goal(chosen)
        self._log_decision(
            chosen,
            scores[chosen.id],
            scores
        )

    def _shutdown_after_exploration(self):
        self.get_logger().info(
            f"ARRET DU NOEUD : stop_reason={self.stop_reason}"
        )
        os._exit(0)

    def _evaluate_candidates(self, candidates):
        scores = {}

        raw_uncertainty = compute_localization_uncertainty(
            self.current_pose_cov
        )

        norm_uncertainty = normalize_uncertainty(
            raw_uncertainty,
            reference=self.uncertainty_reference
        )

        def cout_distance(c):
            """Distance REELLEMENT a parcourir jusqu'au candidat.

            candidate_generator transmet la distance routiere dans
            position.z, calculee par son parcours du graphe. C'est la
            grandeur que le sujet demande pour le critere de cout de
            deplacement.

            La distance a vol d'oiseau, utilisee jusqu'ici, pouvait en
            etre tres eloignee : mesure sur valid_capteur40.log, un
            candidat a 80 m a vol d'oiseau demandait 495 m de route.

            Repli sur la distance euclidienne si z n'est pas renseigne.
            """
            if c.road_distance > 0.0:
                return c.road_distance
            return math.hypot(
                c.x - self.current_x,
                c.y - self.current_y
            )

        max_dist = max(
            (cout_distance(c) for c in candidates),
            default=1.0
        ) or 1.0

        for c in candidates:
            distance = cout_distance(c)

            # /grid_prob_map est publiee par RTAB-Map, donc dans
            # SON repere (miroir lateral, cf. on_pose). C'est
            # donc la coordonnee du candidat qu'on y transpose.
            # Sans cela, la grille etait interrogee a une position
            # miroir, presque toujours hors de la zone deja
            # cartographiee, donc jugee inexploree : le critere
            # saturait a 1.0 pour la quasi-totalite des candidats
            # et ne discriminait plus rien.
            # --- Gain d'information LE LONG DU TRAJET ---
            # L'ancienne formulation n'evaluait que le voisinage du
            # point d'arrivee. Les candidats d'une meme decision etant
            # tous a peu pres a la meme distance, ils tombaient tous du
            # meme cote de la frontiere de cartographie : le critere
            # valait 0.000 pour tous ou 1.000 pour tous, jamais de
            # valeur intermediaire. Le critere qui pese le plus lourd
            # dans la decision (0,30) ne classait rien.
            #
            # On mesure desormais ce que le vehicule decouvrira EN S'Y
            # RENDANT, ce qui est la formulation usuelle en Active SLAM.
            #
            # La transposition en -c.y reste necessaire : la grille vient
            # de RTAB-Map, donc de son repere miroir (cf. on_pose). La
            # position courante subit la meme transposition.
            info_gain = compute_path_information_gain(
                self.current_grid,
                self.current_x,
                -self.current_y,
                c.x,
                -c.y,
                sample_step=self.path_sample_step,
                corridor_radius=self.path_corridor_radius
            )

            # /mapGraph vient egalement de RTAB-Map : meme
            # transposition que pour la grille. Le graphe etait
            # jusqu'ici interroge loin de ses noeuds reels, d'ou
            # un potentiel de fermeture de boucle bloque autour
            # de 0.10 sur toute la campagne.
            loop_closure = compute_loop_closure_potential(
                self.current_map_graph,
                c.x,
                -c.y,
                self.current_node_id
            )

            safety = compute_safety_score(
                self.current_obstacles,
                c.x,
                c.y,
                self.current_x,
                self.current_y
            )

            scores[c.id] = {
                'distance': distance,
                'distance_normalized': distance / max_dist,
                'information_gain': info_gain,
                'loop_closure': loop_closure,
                # Etat global du systeme, identique pour tous les
                # candidats : conserve ici pour le log CSV. C'est
                # select_weighted qui l'exploite, en modulant la
                # ponderation info_gain / loop_closure.
                'localization_uncertainty': norm_uncertainty,
                'safety': safety,
            }

        return scores

    def _publish_goal(self, chosen):
        msg = PoseStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = chosen.x
        msg.pose.position.y = chosen.y
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.last_goal_time = self.get_clock().now()

        self.get_logger().info(
            f"Nouvelle destination choisie "
            f"(strategie={self.strategy}) : "
            f"x={chosen.x:.1f}, y={chosen.y:.1f}"
        )

    def on_collision(self, msg):
        if msg.data in self.collisions:
            self.collisions[msg.data] += 1
        self.get_logger().warn(
            f"EVENEMENT DE SECURITE : collision {msg.data} | "
            f"cumul v={self.collisions['vehicule']} "
            f"p={self.collisions['pieton']} "
            f"o={self.collisions['obstacle']}"
        )

    def _log_decision(self, chosen, score, scores):
        if self._log_writer is None:
            return

        # Statistiques du critere de securite sur tous les candidats
        # evalues. 'safety_min' est le score du candidat le plus expose,
        # 'candidats_penalises' le nombre de candidats dont le score est
        # descendu sous 1.0. Ces deux valeurs mesurent l'INFLUENCE reelle
        # du critere sur le classement : le score du seul candidat retenu
        # ne la mesure pas, puisque la selection tend justement a retenir
        # les candidats surs.
        safety_values = [s['safety'] for s in scores.values()]
        nb_candidats = len(safety_values)
        safety_min = min(safety_values) if safety_values else 1.0
        candidats_penalises = sum(1 for v in safety_values if v < 1.0)

        # Meme raisonnement que pour la securite, applique aux deux
        # criteres exploratoires : c'est l'ETENDUE sur l'ensemble des
        # candidats qui dit si un critere classe quelque chose. Le
        # score du seul candidat retenu ne le dit pas.
        ig_values = [s['information_gain'] for s in scores.values()]
        lc_values = [s['loop_closure'] for s in scores.values()]
        info_gain_min = min(ig_values) if ig_values else 0.0
        info_gain_max = max(ig_values) if ig_values else 0.0
        loop_closure_max = max(lc_values) if lc_values else 0.0

        self.get_logger().info(
            f"SECURITE : {candidats_penalises}/{nb_candidats} candidats "
            f"penalises | safety_min={safety_min:.4f} | "
            f"safety_retenu={score['safety']:.4f}"
        )

        self.get_logger().info(
            f"CRITERES sur {nb_candidats} candidats : "
            f"info_gain [{info_gain_min:.3f} - {info_gain_max:.3f}] "
            f"etendue={info_gain_max - info_gain_min:.3f} | "
            f"loop_closure max={loop_closure_max:.3f}"
        )

        self._log_writer.writerow([
            self.get_clock().now().to_msg().sec,
            self.strategy,
            chosen.x,
            chosen.y,
            score['information_gain'],
            score['loop_closure'],
            score['localization_uncertainty'],
            score['safety'],
            score['distance'],
            self.collisions['vehicule'],
            self.collisions['pieton'],
            self.collisions['obstacle'],
            nb_candidats,
            safety_min,
            candidats_penalises,
            info_gain_min,
            info_gain_max,
            loop_closure_max,
        ])

        self._log_fh.flush()

    def destroy_node(self):
        if self._log_fh is not None:
            self._log_fh.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DecisionMaker()

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
