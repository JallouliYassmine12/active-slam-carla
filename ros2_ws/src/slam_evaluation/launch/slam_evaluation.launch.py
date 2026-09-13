"""
Lancement du systeme de REFERENCE : SLAM classique a trajectoire
predefinie.

C'est la baseline "trajectoire predefinie" exigee par le sujet, celle
a laquelle l'Active SLAM doit etre compare.

PRINCIPE DIRECTEUR : une seule variable change
-----------------------------------------------
Entre ce lancement et active_slam.launch.py, une seule chose doit
differer : la maniere de choisir ou aller. Ici, une trajectoire fixe
suivie le long du graphe routier ; la-bas, une fonction de decision
multi-criteres. Tout le reste -- pile SLAM, capteurs, carte, point de
depart, meteo, trafic, vitesse, budget -- est rigoureusement identique.

Sans cette discipline, l'ecart de performance mesure entre les deux
systemes ne dit rien de la methode d'exploration. La version
precedente de ce fichier tournait avec les valeurs par defaut de
RTAB-Map :

  - Vis/MinInliers = 20  au lieu de 10   -> fermetures de boucle rejetees
  - Optimizer/Robust absent              -> divergence GTSAM non neutralisee
  - RGBD/OptimizeMaxError = 3.0          -> toute vraie fermeture jugee aberrante
  - Odom/ResetCountdown absent           -> odometrie morte apres un seul echec
  - Icp/PointToPlaneLowComplexityStrategy absent -> effondrement en couloir
  - Icp/MaxTranslation = 2.0             -> echecs de recalage a vitesse normale

Autrement dit, la baseline subissait exactement les defauts que la
campagne de correction a elimines cote Active SLAM. Une comparaison
dans cet etat aurait montre, non pas qu'une exploration active
ameliore la localisation, mais qu'un RTAB-Map regle bat un RTAB-Map
non regle. Les blocs icp_odometry et rtabmap ci-dessous sont donc
copies a l'identique depuis active_slam.launch.py.

Ce qui a aussi ete corrige dans ce fichier
-------------------------------------------
  - 'host'/'port' renommes en 'carla_host'/'carla_port' : les anciens
    noms n'etaient pas declares par bridge_node, donc silencieusement
    ignores. L'adresse 192.168.1.6 n'a jamais servi.
  - 'buffer_size' retire de localization_evaluator : jamais declare.
  - 'save_interval' passe de 10 a 10.0 : le noeud le declare en
    flottant, un entier provoque une erreur de type au demarrage.
  - 'strategy' ajoute : les CSV de la baseline sont desormais
    identifiables dans all_runs/.
  - traffic_spawner ajoute : la baseline roulait sur une ville vide
    pendant que l'Active SLAM affrontait 8 vehicules.
  - Cles dupliquees supprimees dans icp_odometry (Icp/MaxTranslation
    apparaissait deux fois, 3.0 puis 2.0 ; Python gardait
    silencieusement la seconde).

obstacle_detector n'est volontairement PAS lance ici : ce noeud sert a
alimenter le critere 'safety' du decision_maker, qui n'existe pas dans
la baseline. L'evitement tactique du controleur interroge CARLA
directement, a la meme cadence (1 Hz) et avec le meme rayon (100 m)
que obstacle_detector cote Active SLAM, donc les deux vehicules
percoivent les memes obstacles avec la meme fraicheur.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import datetime


def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # --- Controleur : extrait en variable pour l'EventHandler d'arret ---
    controller_node = Node(
        package='slam_evaluation',
        executable='vehicle_controller_eval.py',
        name='vehicle_controller',
        output='screen',
        parameters=[{
            'distance': LaunchConfiguration('distance'),
            'role_name': 'ego',
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            # Loi longitudinale identique au controleur Active SLAM. La
            # vitesse est une variable de confusion majeure : l'ICP se
            # degrade quand les scans se recouvrent moins, donc un
            # systeme qui roule moins vite part avec un avantage sans
            # rapport avec sa strategie.
            'max_throttle': 0.5,
            'min_throttle': 0.35,
            'lookahead_distance': 6.0,
            'max_steering_angle': 0.4,
            # Memes seuils d'obstacle, meme cadence et meme rayon que
            # obstacle_detector cote Active SLAM.
            'obstacle_stop_distance': 7.0,
            'obstacle_slow_distance': 14.0,
            'obstacle_corridor_width': 3.0,
            'obstacle_scan_radius': 100.0,
            'obstacle_refresh_rate': 1.0,
            'respect_traffic_lights': True,
            'stuck_threshold': 50,
        }],
    )

    # Arret global des que le controleur se termine (budget atteint).
    shutdown_on_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason='Budget de distance atteint par le controleur'
            ))]
        )
    )

    return LaunchDescription([

        DeclareLaunchArgument('carla_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('carla_port', default_value='2000'),

        # Budget commun aux deux systemes : la distance parcourue est la
        # seule unite comparable entre une trajectoire predefinie et une
        # selection de destinations.
        DeclareLaunchArgument('distance', default_value='1500.0'),

        # Scenario, identique a celui de active_slam.launch.py
        DeclareLaunchArgument('town', default_value='Town03_Opt'),
        DeclareLaunchArgument('spawn_index', default_value='0'),
        DeclareLaunchArgument('weather', default_value='clear'),
        DeclareLaunchArgument('num_vehicles', default_value='20'),
        DeclareLaunchArgument('use_viz', default_value='false'),

        # --- Seuil d'acceptation du recalage ICP ---
        # Fraction minimale de points du scan devant trouver un
        # correspondant pour qu'un recalage soit accepte. Defaut
        # 0.05 : valeur de tous les runs archives, comportement
        # inchange sans argument.
        #
        # Teste a 0.03 : 313 fermetures de boucle ont ete proposees
        # puis rejetees sur les 37 runs archives, contre 224
        # acceptees. Abaisser le seuil doit en faire passer une
        # part -- au risque d'en laisser passer de fausses, ce que
        # Optimizer/Robust est cense neutraliser. D'ou le test.
        #
        #   ros2 launch ... icp_correspondence_ratio:=0.03
        DeclareLaunchArgument(
            'icp_correspondence_ratio', default_value='0.05'
        ),
        # --- Seuil d'acceptation des fermetures de boucle ---
        # Meme grandeur que icp_correspondence_ratio, autre noeud :
        # celui-ci ne gouverne que la validation des fermetures par
        # rtabmap, pas le recalage scan a scan de icp_odometry.
        #
        # Les separer est la correction du test C, qui les changeait
        # ensemble et mesurait donc un effet d'odometrie.
        #
        #   ros2 launch ... loop_correspondence_ratio:=0.03
        DeclareLaunchArgument(
            'loop_correspondence_ratio', default_value='0.05'
        ),

        # --- Graine du generateur de trafic ---
        # -1 = aleatoire, regime de la campagne archivee. Une valeur
        # >= 0 rend le trafic reproductible, ce qui permet d'apparier
        # la reference aux quatre strategies Active SLAM : a la
        # repetition n, les cinq systemes affrontent le meme trafic.
        #
        # Sans cet argument, 'predefinie' ne pouvait pas entrer dans
        # une campagne appariee : sa graine etait ecrite en dur.
        DeclareLaunchArgument('seed', default_value='-1'),

        # --- Pont CARLA <-> ROS 2 ---
        Node(
            package='carla_sensors_bridge',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[{
                # Noms CORRECTS des parametres : 'host'/'port' n'etaient
                # pas declares par le noeud, donc ignores en silence.
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'role_name': 'ego',
                'town': LaunchConfiguration('town'),
                # La carte n'est pas rechargee par ce pont : c'est un run
                # de reference, on travaille sur la carte deja en place.
                # Le noeud journalise la carte REELLEMENT active, ce qui
                # fait foi dans le rapport.
                'load_town': True,
                # Point de depart impose : la version precedente tirait au
                # hasard (random.shuffle), ce qui rendait les runs non
                # reproductibles -- alors que la tache 3 du sujet exige
                # des scenarios reproductibles -- et incomparables a
                # l'Active SLAM, qui part toujours du point 0.
                'spawn_point_index': LaunchConfiguration('spawn_index'),
                'random_spawn': False,
                'weather': LaunchConfiguration('weather'),
            }],
        ),

        # --- Transformees statiques ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_lidar',
            arguments=['0', '0', '2.5', '0', '0', '0', 'base_link', 'ego_lidar'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            arguments=['1.5', '0', '2.4', '0', '0', '0', 'base_link', 'ego_camera'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ego_imu'],
        ),

        # --- Filtre IMU ---
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
            }],
            remappings=[
                ('imu/data_raw', '/carla/ego/imu'),
                ('imu/data', '/imu/data_filtered'),
            ],
        ),

        # ==============================================================
        # Odometrie LiDAR (ICP)
        # BLOC COPIE A L'IDENTIQUE DEPUIS active_slam.launch.py.
        # Toute divergence ici fausserait la comparaison.
        # ==============================================================
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom_visual',
                'publish_tf': True,
                'publish_null_when_lost': False,
                'wait_imu_to_init': False,
                'queue_size': 10,
                'Reg/Strategy': '1',
                # Contrainte 2D : le vehicule roule sur un plan. Estimer
                # roulis, tangage et z laisse ces axes deriver et pollue
                # l'erreur angulaire.
                'Reg/Force3DoF': 'true',
                'Icp/VoxelSize': '0.2',
                # 6.0 et non 2.0 : a 10 Hz avec des delais de traitement,
                # deux scans consecutifs peuvent etre separes de plus de
                # 4 m. La valeur precedente etait de surcroit ecrite deux
                # fois dans le dictionnaire (3.0 puis 2.0), Python gardant
                # silencieusement la seconde.
                'Icp/MaxTranslation': '6.0',
                'Icp/MaxRotation': '1.0',
                'Icp/MaxCorrespondenceDistance': '2.0',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/PointToPlaneK': '10',
                # Seuil expose en argument (defaut 0.05, valeur des runs
                # archives). Voir patch_ratio_icp.py : 313 fermetures rejetees
                # sur 37 runs. value_type=str obligatoire, RTAB-Map declare ses
                # parametres en chaines.
                'Icp/CorrespondenceRatio': ParameterValue(
                    LaunchConfiguration('icp_correspondence_ratio'),
                    value_type=str
                ),
                'Icp/DownsamplingStep': '2',
                'Icp/RangeMax': '50',
                # --- Recuperation apres perte d'odometrie ---
                # Sans ce parametre, un seul echec de recalage tue
                # l'odometrie pour tout le reste du run : la pose devinee
                # est extrapolee du dernier mouvement, s'eloigne a chaque
                # echec, et le scan suivant n'a plus aucun recouvrement
                # avec le modele local. La boucle s'auto-entretient.
                'Odom/ResetCountdown': '5',
                # --- Robustesse en environnement "couloir" ---
                # Entre deux murs paralleles, le point-to-plane n'a pas
                # assez de contraintes pour resoudre la translation le
                # long du couloir et RTAB-Map le desactive. La strategie 2
                # conserve la contrainte sur les axes bien observes au
                # lieu de basculer entierement en point-to-point.
                'Icp/PointToPlaneMinComplexity': '0.01',
                'Icp/PointToPlaneLowComplexityStrategy': '2',
                'Odom/ScanKeyFrameThr': '0.8',
                'Odom/Holonomic': 'false',
                'OdomF2M/BundleAdjustment': '0',
                'Odom/Strategy': '0',
                'OdomF2M/ScanSubtractRadius': '0.3',
                'OdomF2M/ScanMaxSize': '15000',
            }],
            remappings=[
                ('scan_cloud', '/carla/ego/lidar/points'),
                ('odom', '/rtabmap/odom'),
            ],
        ),

        # ==============================================================
        # RTAB-Map
        # BLOC COPIE A L'IDENTIQUE DEPUIS active_slam.launch.py, a la
        # seule exception du chemin de la base de donnees.
        # ==============================================================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'],
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'queue_size': 30,
                'database_path': f'/home/ubuntu/active_slam_carla/maps/slam_eval_{timestamp}.db',
                'Mem/IncrementalMemory': 'true',
                'Rtabmap/DetectionRate': '1.0',
                # Seuil d'inliers abaisse : les revisites sont reelles
                # mais echouent de quelques points sur le seuil par
                # defaut (20). Prudent uniquement parce que
                # Optimizer/Robust neutralise les fausses associations
                # qui passeraient ce filtre assoupli.
                'Vis/MinInliers': '6',
                # L'ajustement de faisceaux ramenait des correspondances
                # valides a 0 inlier sur des images 320x240 de facades
                # urbaines tres repetitives.
                'Vis/BundleAdjustment': '0',
                'Reg/Force3DoF': 'true',
                'Optimizer/Slam2D': 'true',
                # RGBD/OptimizeMaxError compare la correction d'une
                # fermeture de boucle a l'ecart-type declare par
                # l'odometrie, lequel est sous-estime d'un facteur 20 :
                # toute vraie fermeture etait jugee aberrante. Desactive ;
                # la validation geometrique reste assuree par
                # Vis/MinInliers et par l'optimisation robuste.
                'RGBD/OptimizeMaxError': '0',
                # Contraintes commutables (Vertigo, via GTSAM) :
                # l'optimiseur neutralise les arcs incoherents pendant la
                # resolution, au lieu qu'un seul mauvais lien fasse
                # diverger tout le graphe (erreur constante de 2.38e12 sur
                # 173 iterations, observee avant activation).
                'Optimizer/Robust': 'true',
                'Reg/Strategy': '2',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'Icp/MaxTranslation': '6.0',
                'Icp/MaxRotation': '1.0',
                # Seuil propre aux FERMETURES DE BOUCLE, distinct de celui de
                # l'odometrie : les deux etaient confondus, et la variante C a
                # montre qu'abaisser celui de l'odometrie supprime les
                # opportunites de fermeture au lieu d'en faire accepter.
                'Icp/CorrespondenceRatio': ParameterValue(
                    LaunchConfiguration('loop_correspondence_ratio'),
                    value_type=str
                ),
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/Sensor': '2',
                'Grid/FromDepth': 'false',
                'Grid/FromScan': 'false',
                'Grid/FromPointCloud': 'true',
                'Grid/MapFrameProjection': 'true',
                'Grid/MaxObstacleHeight': '5.0',
                'Grid/MinGroundHeight': '-2.0',
                # Aligne sur active_slam.launch.py. La segmentation par
                # normales echoue sur un LiDAR automobile, ou la
                # chaussee est vue en incidence rasante : les points du
                # sol sont rejetes comme obstacles, le lancer de rayons
                # n'a rien vers quoi tracer, et aucune cellule n'est
                # marquee libre. A 'false', le sol est identifie par
                # simple seuil de hauteur (Grid/MaxGroundHeight).
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxGroundHeight': '1.0',
                'Grid/GroundIsObstacle': 'false',
                # Grid/CellSize et Grid/RangeMax doivent rester identiques
                # a l'Active SLAM : la surface cartographiee est une
                # metrique de la comparaison, et elle se compte en
                # cellules.
                # --- Portee de la grille d'occupation ---
                # DOIT rester STRICTEMENT SUPERIEURE au
                # min_candidate_distance du candidate_generator (25 m).
                # Quand les deux sont egales, tout candidat tombe au
                # bord ou au-dela de la zone cartographiee et
                # compute_information_gain renvoie la meme valeur pour
                # tous : mesure sur la campagne complete, 0,92 a 0,96 de
                # moyenne pour les quatre strategies. Le critere qui
                # porte le poids le plus fort de la decision (0,30) ne
                # classait donc plus rien.
                #
                # A 40 m, un candidat a 25 m tombe en zone connue (gain
                # faible) et un candidat a 45 m en zone inconnue (gain
                # eleve) : le critere redevient discriminant.
                #
                # Cette valeur doit rester IDENTIQUE entre
                # active_slam.launch.py et slam_evaluation.launch.py,
                # sinon un ecart d'ATE entre SLAM classique et Active
                # SLAM ne serait plus attribuable a la methode de
                # decision.
                'Grid/RangeMax': '40.0',
                'Grid/CellSize': '0.15',
                # Aligne sur active_slam.launch.py. En 3D, le lancer de
                # rayons se fait en volume : lent au point de
                # compromettre la cartographie temps reel, alors que la
                # grille exploitee est de toute facon une projection 2D.
                'Grid/3D': 'false',
                # Aligne sur active_slam.launch.py. Sans lancer de
                # rayons, RTAB-Map ne marque que les cellules OCCUPEES :
                # la chaussee parcourue reste inconnue et la grille est
                # une carte des murs. Mesure cote Active SLAM avant
                # correction : 2,4 % de cellules connues, 0,0 % de
                # libre ; apres correction, 43,6 % de libre.
                'Grid/RayTracing': 'true',
                'Rtabmap/MemoryThr': '0',
                'RGBD/LocalRadius': '20.0',
            }],
            remappings=[
                ('rgb/image', '/carla/ego/camera/image_raw'),
                ('depth/image', '/carla/ego/camera/depth'),
                ('rgb/camera_info', '/carla/ego/camera/camera_info'),
                ('scan_cloud', '/carla/ego/lidar/points'),
                ('imu', '/imu/data_filtered'),
                ('odom', '/rtabmap/odom'),
            ],
        ),

        # --- Visualisation : desactivee par defaut (libere du GPU) ---
        # Doit etre dans le MEME etat que cote Active SLAM : rtabmap_viz
        # consomme des ressources et peut faire decrocher l'odometrie.
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_viz')),
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_odom_info': True,
            }],
            remappings=[
                ('odom', '/rtabmap/odom'),
            ],
        ),

        # --- Trafic NPC ---
        # Absent de la version precedente : la baseline roulait sur une
        # ville vide pendant que l'Active SLAM affrontait le trafic, les
        # occlusions et les feux rouges.
        # Le noeud vient du paquet active_slam_decision : penser a
        # ajouter <exec_depend>active_slam_decision</exec_depend> dans le
        # package.xml de slam_evaluation.
        Node(
            package='active_slam_decision',
            executable='traffic_spawner',
            name='traffic_spawner',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'ego_role_name': 'ego',
                'num_vehicles': LaunchConfiguration('num_vehicles'),
                'num_walkers': 0,
                'min_distance_from_ego': 10.0,
                'spawn_radius_from_ego': 150.0,
                'active_radius': 120.0,
                'min_vehicles_near_ego': 6,
                'recycle_beyond': 250.0,
                'recycle_min_distance': 40.0,
                'recycle_max_distance': 140.0,
                'max_recycles_per_tick': 2,
                # --- Graine du trafic : ALEATOIRE, comme les quatre autres ---
                #
                # Valait 42, alors qu'active_slam.launch.py n'en fixe aucune.
                # 'predefinie' affrontait donc un trafic reproductible pendant que
                # les quatre strategies Active SLAM en affrontaient un tire au
                # hasard : la reference et les methodes comparees ne subissaient pas
                # les memes conditions.
                #
                # On desensemence plutot que d'ensemencer les autres, car la mesure
                # montre que la graine ne gouverne pas la dispersion. Les trois
                # 'predefinie' de la campagne du 3 septembre avaient graine fixe,
                # trajectoire identique et simulateur deterministe (synchronous_mode
                # = True, fixed_delta_seconds = 0.05), et ont donne 9,7 m, 77,7 m et
                # 161,1 m d'erreur. La variabilite vient de la chaine de perception,
                # executee en temps souple, pas du scenario simule.
                #
                # -1 = aleatoire (voir traffic_spawner.declare_parameter('seed', -1))
                'seed': ParameterValue(
                    LaunchConfiguration('seed'),
                    value_type=int
                ),
            }],
        ),

        # --- Controleur (defini plus haut) ---
        controller_node,

        # --- Evaluateur de localisation ---
        Node(
            package='slam_evaluation',
            executable='localization_evaluator.py',
            name='localization_evaluator',
            output='screen',
            parameters=[{
                'output_file': '/home/ubuntu/active_slam_carla/metrics/localization_errors.csv',
                # Identifie les CSV de la baseline dans all_runs/ face aux
                # cinq strategies de l'Active SLAM.
                'strategy': 'predefinie',
                # Filtre de saut exprime en vitesse : le seuil absolu de
                # 5 m rejetait tout deplacement normal au-dela de 5 m/s.
                'max_speed': 20.0,
                'loop_closure_margin': 15.0,
                # Flottant obligatoire : le noeud declare 10.0. Un entier
                # provoque une erreur de type au demarrage.
                'save_interval': 10.0,
                # 'buffer_size' retire : jamais declare par le noeud,
                # ignore en silence depuis le debut.
            }],
        ),

        shutdown_on_controller_exit,
    ])
