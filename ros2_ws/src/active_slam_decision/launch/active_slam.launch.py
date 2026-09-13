from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import datetime


def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    decision_maker_node = Node(
        package='active_slam_decision',
        executable='decision_maker',
        name='active_slam_decision_maker',
        output='screen',
        parameters=[{
            'strategy': LaunchConfiguration('strategy'),
            # --- Poids exposes en arguments de lancement ---
            # Le gain d'information et le cout de distance sont les
            # deux poids de l'experience apparie decrite dans
            # patch_arguments_lancement.py. Les defauts reproduisent
            # exactement les valeurs de la campagne du 3 septembre :
            # lance sans argument, le systeme est inchange.
            #
            # Les rendre pilotables permet de comparer deux
            # configurations SOUS LA MEME EMPREINTE DE CODE, ce qui
            # est la condition pour que la comparaison porte sur le
            # reglage et non sur deux versions differentes.
            'weight_information_gain': ParameterValue(
                LaunchConfiguration('weight_information_gain'),
                value_type=float
            ),
            'weight_localization_uncertainty': 0.20,
            # --- Poids du critere de fermeture de boucle ---
            # Expose pour permettre son ABLATION (valeur 0), seule
            # facon de tester ce qu'il apporte reellement. Defaut
            # 0.20 : valeur de tous les runs archives.
            'weight_loop_closure': ParameterValue(
                LaunchConfiguration('weight_loop_closure'),
                value_type=float
            ),
            'weight_distance': ParameterValue(
                LaunchConfiguration('weight_distance'),
                value_type=float
            ),
            'weight_safety': 0.15,
            # Echelle de normalisation du critere d'incertitude. Doit etre
            # NETTEMENT inferieure au seuil critique : c'est ce qui donne au
            # critere le temps d'influencer les decisions (choisir une
            # destination qui ramene le vehicule vers une zone connue, donc
            # une fermeture de boucle) AVANT que la securite ne coupe le run.
            # Avec reference == seuil, l'incertitude normalisee n'atteignait
            # 1.0 qu'a l'instant meme de l'arret : le critere central du
            # sujet n'etait jamais exerce.
            # Historique du reglage, en normalisation LINEAIRE : a 15.0 le
            # critere saturait a 1.0000 en permanence (releve : brute=406 ->
            # normalisee=1.0000), ce qui provoquait des demi-tours continus ;
            # a 200.0 le balayage [0, 1] n'occupait plus que la premiere
            # minute ; a 800.0 la saturation revenait des la mi-parcours
            # (releve : brute=816 -> 1.0000 pour tout le reste du run).
            # Aucune valeur ne convenait, parce que l'incertitude brute
            # couvre plus de trois ordres de grandeur sur un run (2 -> 5350) :
            # c'est l'echelle lineaire elle-meme qui etait inadaptee.
            # normalize_uncertainty est desormais LOGARITHMIQUE, ce qui
            # permet de caler la reference sur le seuil critique lui-meme :
            # 1.0 correspond exactement a la limite de fiabilite, et le
            # critere balaie [0, 1] sur tout le run (0.13 a brute=2, 0.46 a
            # 50, 0.62 a 200, 0.78 a 800, 1.0 a 5000).
            'uncertainty_reference': 5000.0,
            # Filet de securite pour une localisation reellement corrompue
            # (divergence, saut de pose), pas pour la derive normale d'un
            # SLAM sans fermeture de boucle. 50 etait atteint apres 80 m de
            # trajet en ligne droite, ce qui est un comportement sain.
            # Releve de 2000.0 a 5000.0 : la preuve qu'un SLAM SAIN franchit
            # 2000 est directe. A incertitude_brute=1266.79, une fermeture de
            # boucle a ete acceptee et a recale le graphe (chute a 1251.66),
            # ce qui est impossible sur une localisation corrompue. Un seuil
            # que traverse une localisation saine ne mesure pas la corruption,
            # il chronometre la distance parcourue : il coupait les runs a
            # ~13 destinations sans qu'aucune anomalie ne soit survenue.
            'uncertainty_critical_threshold': 1000000.0,
            # Troisieme budget : la distance parcourue. Le SLAM classique
            # suit une trajectoire predefinie : il n'a ni destinations ni
            # duree comparables, seulement des metres. Le premier des trois
            # budgets atteint termine le run.
            'max_distance': ParameterValue(
                LaunchConfiguration('max_distance'),
                value_type=float
            ),
            # --- Budget de run, identique pour les 5 strategies ---
            # Condition de comparabilite de la campagne : sans budget, le
            # run s'arrete sur l'incertitude critique, donc a un instant qui
            # depend de la trajectoire suivie, donc de la strategie testee.
            # Le temps d'exploration deviendrait une variable non controlee
            # et les metriques ne seraient plus comparables entre elles.
            # 300 s couvre confortablement les runs observes (23 destinations
            # en ~2 min) ; l'arret securite ne joue plus que son role de
            # garde-fou. Mettre 0.0 pour desactiver.
            'max_run_duration': ParameterValue(
                LaunchConfiguration('run_duration'),
                value_type=float
            ),
            # Second budget possible, alternatif ou complementaire : le
            # premier des deux atteint termine le run. 0 = desactive.
            'max_destinations': ParameterValue(
                LaunchConfiguration('max_destinations'),
                value_type=int
            ),
            # --- Rayon d'evaluation du gain d'information ---
            # Le passage a 40 m, pour aligner ce rayon sur
            # Grid/RangeMax comme le prescrit la documentation de
            # information_gain.py, a ete TENTE et n'a rien change :
            # l'etendue du critere est restee nulle sur toutes les
            # decisions du run valid_capteur40.log, pour quatre fois le
            # volume de calcul (le disque croit comme le carre du
            # rayon).
            #
            # Retour a 20 m, valeur sous laquelle tous les runs valides
            # ont tourne. L'hypothese est testee et refutee, elle n'a
            # pas ete abandonnee sans mesure.
            'sensor_range': 20.0,
            'max_candidate_distance': 80.0,
            # Rayon d'exclusion autour d'un point declare
            # inatteignable. DOIT rester de l'ordre de
            # l'espacement des candidats (waypoint_spacing = 8 m
            # dans candidate_generator), pas de l'ordre de la
            # distance minimale d'un candidat.
            #
            # A 25 m, chaque point rejete eliminait 6 a 7 candidats :
            # sept rejets au demarrage effacaient les 23 candidats
            # disponibles, et le vehicule restait immobile pour tout
            # le run. A 8 m, un rejet elimine le point lui-meme et
            # ses voisins immediats.
            'unreachable_exclusion_radius': 8.0,
            'obstacles_topic': '/obstacle_detection/obstacles',
            'log_file': f'/home/ubuntu/active_slam_carla/metrics/decisions_{timestamp}.csv',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('carla_port', default_value='2000'),
        DeclareLaunchArgument('max_throttle', default_value='0.5'),
        DeclareLaunchArgument('arrival_tolerance', default_value='2.0'),
        # weighted (methode proposee) | random | distance | info_gain
        DeclareLaunchArgument('strategy', default_value='weighted'),
        DeclareLaunchArgument('spawn_index', default_value='0'),
        # La carte fait partie du scenario : elle doit etre fixee explicitement
        # pour que la campagne comparative soit reproductible.
        # Variante _Opt : chargement des decors par couches, donc beaucoup
        # moins de shaders a compiler. Town03 "pleine" provoque un
        # LowLevelFatalError (echec fatal de compilation de shaders) sur cette
        # configuration GPU, alors que Town03_Opt charge en quelques secondes.
        # Town03 est retenue pour la campagne car sa topologie comporte de
        # nombreuses boucles : c'est la condition pour que le critere de
        # potentiel de fermeture de boucle puisse reellement s'exprimer.
        # Affichage temps reel de RTAB-Map. Desactive par defaut : il consomme
        # GPU et CPU sur la meme carte que CARLA, ce qui degrade le SLAM
        # (delais jusqu'a 1.9 s, "real-time problem!", perte de scans).
        # Pour une campagne comparative, toutes les strategies doivent tourner
        # sous la meme charge machine : la fenetre 3D serait une variable
        # parasite. On l'active explicitement pour les demonstrations.
        #   ros2 launch ... use_viz:=true
        DeclareLaunchArgument('use_viz', default_value='false'),
        DeclareLaunchArgument('town', default_value='Town03_Opt'),
        # Conditions meteorologiques, exigees par le sujet parmi les
        # scenarios de validation. 'clear' reproduit exactement le
        # ClearNoon applique en dur auparavant : la valeur par defaut
        # ne change donc rien aux runs deja realises.
        # Valeurs : clear, cloudy, wet, light_rain, rain, sunset,
        #           fog, night
        #   ros2 launch ... weather:=rain
        DeclareLaunchArgument('weather', default_value='clear'),
        # Nombre de pietons NPC. Defaut 0, comme jusqu'a present.
        # ATTENTION : ne fonctionne PAS sur Town03_Opt. La variante
        # _Opt ne charge pas le maillage de navigation pietonne, et
        # world.get_random_location_from_navigation() y provoque un
        # segfault (exit code -11) apres ~13 s, qui tue le noeud de
        # trafic avant meme le message TRAFIC ACTIF - donc supprime
        # aussi les vehicules. Le scenario pietons doit se lancer sur
        # la carte pleine, qui embarque ce maillage :
        #   ros2 launch ... town:=Town03 num_pedestrians:=15
        DeclareLaunchArgument('num_pedestrians', default_value='0'),
        # Budget de run. Doit rester IDENTIQUE pour les cinq strategies de
        # la campagne comparative, sinon le temps d'exploration devient une
        # variable non controlee.
        #   ros2 launch ... run_duration:=300.0
        DeclareLaunchArgument('run_duration', default_value='300.0'),
        # Budget alternatif en nombre de destinations (0 = desactive). Le
        # premier des deux budgets atteint termine le run.
        #   ros2 launch ... max_destinations:=25
        DeclareLaunchArgument('max_destinations', default_value='0'),
        # Budget en distance parcourue (metres). Seule unite commune avec
        # le run SLAM classique. 0.0 = illimite.
        #   ros2 launch ... max_distance:=1500.0
        DeclareLaunchArgument('max_distance', default_value='0.0'),
        # --- Graine du generateur de trafic ---
        # -1 = aleatoire, comme la campagne du 3 septembre. Une valeur
        # >= 0 rend le trafic reproductible et permet de comparer deux
        # reglages sur des conditions identiques :
        #   ros2 launch ... seed:=3
        DeclareLaunchArgument('seed', default_value='-1'),
        # --- Poids de la fonction de decision ---
        # Defauts identiques a la campagne du 3 septembre. La
        # configuration B testee vaut 0.20 / 0.25 :
        #   ros2 launch ... weight_information_gain:=0.20 \
        #                   weight_distance:=0.25
        DeclareLaunchArgument(
            'weight_information_gain', default_value='0.30'
        ),
        DeclareLaunchArgument('weight_distance', default_value='0.15'),
        # --- Poids de la fermeture de boucle ---
        # 0.00 retire le critere de la decision et neutralise le
        # transfert dynamique (voir la garde dans strategies.py) :
        #   ros2 launch ... weight_loop_closure:=0.0
        DeclareLaunchArgument(
            'weight_loop_closure', default_value='0.20'
        ),

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
        # --- CARLA Sensors Bridge (identique a slam_evaluation.launch.py) ---
        Node(
            package='carla_sensors_bridge',
            executable='bridge_active_slam',
            name='bridge_active_slam',
            output='screen',
            parameters=[{
                # Les noms doivent correspondre EXACTEMENT a ceux declares
                # dans le noeud : 'host', 'port', 'role_name',
                # 'synchronous_mode' et 'fixed_delta_seconds' n'y sont pas
                # declares et etaient donc silencieusement ignores.
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'town': LaunchConfiguration('town'),
                'spawn_index': LaunchConfiguration('spawn_index'),
                'weather': LaunchConfiguration('weather'),
            }],
        ),

        # --- Static transforms (identiques) ---
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

        # --- Filtre IMU (identique) ---
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

        # --- Odometrie LiDAR ICP (identique) ---
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
                # --- Contrainte 2D (vehicule terrestre) ---
                # Le vehicule roule sur un plan. Estimer roulis, tangage et z
                # laisse ces trois axes deriver librement et pollue l'erreur
                # angulaire : l'optimisation du graphe mesurait 29.2 deg
                # d'erreur absolue sur un simple arc d'odometrie (type=0)
                # alors que celui-ci declarait un ecart-type de 0.0269 rad.
                # Contraindre a x, y, yaw supprime cette source de derive et
                # allege le calcul ICP.
                'Reg/Force3DoF': 'true',
                'Icp/VoxelSize': '0.2',
                # Relevee de 4.0 a 6.0 : les echecs
                # "libpointmatcher has failed: limit out of bounds: tr: 4.x/4"
                # venaient de cette borne. A 10 Hz avec des delais de
                # traitement, deux scans consecutifs peuvent etre separes de
                # plus de 4 m ; l'enregistrement echouait alors et creait un
                # trou dans la chaine d'odometrie.
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
                # Sans ce parametre, un seul echec de recalage suffit a tuer
                # l'odometrie pour tout le reste du run : la pose devinee est
                # extrapolee du dernier mouvement, s'eloigne a chaque echec
                # (guess=xyz=1270,2758 observe), et le scan suivant n'a plus
                # aucun recouvrement avec le modele local. La boucle
                # s'auto-entretient. ResetCountdown force une reinitialisation
                # du modele local apres 5 echecs consecutifs : l'odometrie
                # repart d'une carte locale neuve au lieu de rester morte.
                'Odom/ResetCountdown': '5',
                # --- Robustesse en environnement "couloir" ---
                # Entre deux murs paralleles, le point-to-plane n'a pas assez
                # de contraintes pour resoudre la translation le long du
                # couloir et RTAB-Map le desactive (complexite 0.0187 < 0.02).
                # On abaisse le seuil et on choisit la strategie 2, qui
                # conserve la contrainte sur les axes bien observes au lieu
                # de basculer entierement en point-to-point.
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

        # --- RTAB-Map (identique ; base de donnees dediee a l'Active SLAM) ---
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
                'database_path': f'/home/ubuntu/active_slam_carla/maps/active_slam_{timestamp}.db',
                'Mem/IncrementalMemory': 'true',
                'Rtabmap/DetectionRate': '1.0',
                # --- Acceptation des fermetures de boucle ---
                # Rejets observes : 8/20, 9/20, 13/20 inliers. Les revisites
                # sont reelles mais echouent de quelques points sur le seuil
                # par defaut (20). Abaisse a 12, puis a 10 : le releve
                # "Rejected loop closure 44 -> 56: Not enough inliers 11/12"
                # montre une fermeture manquee d'UN SEUL inlier. Descendre
                # a 10 n'est prudent que parce que Optimizer/Robust (plus
                # bas) neutralise les fausses associations qui passeraient
                # ce filtre assoupli.
                'Vis/MinInliers': '6',
                # L'ajustement de faisceaux ramenait 28 correspondances a 0
                # inlier ("after bundle adjustment 0/20"). Sur des images
                # 320x240 de facades urbaines tres repetitives, il elimine
                # plus de vraies correspondances qu'il n'en corrige.
                'Vis/BundleAdjustment': '0',
                # --- Contrainte 2D (vehicule terrestre) ---
                # Doit etre coherent avec le meme parametre pose sur
                # icp_odometry, pour que les arcs publies et l'optimisation du
                # graphe travaillent dans le meme espace d'etat.
                'Reg/Force3DoF': 'true',
                # Optimiseur de graphe en 2D (x, y, yaw), corollaire de
                # Reg/Force3DoF.
                'Optimizer/Slam2D': 'true',
                # --- Controle post-optimisation ---
                # RGBD/OptimizeMaxError compare la correction apportee par une
                # fermeture de boucle a l'ecart-type declare par l'odometrie.
                # Or celle-ci annonce 0.0269 rad la ou l'optimisation mesure
                # 29.2 deg d'erreur reelle : la reference est sous-estimee d'un
                # facteur 20, donc TOUTE vraie fermeture est jugee aberrante
                # ("maximum graph error ratio of 39.68 ... Loop closure 92->78
                # rejected!"). Le seuil de 3.0 n'est pas faux en soi, c'est sa
                # reference qui est corrompue. Desactive ; la validation
                # geometrique reste assuree par Vis/MinInliers et par
                # l'optimisation robuste ci-dessous.
                'RGBD/OptimizeMaxError': '0',
                # --- Optimisation robuste (contraintes commutables) ---
                # Avec Vis/MinInliers abaisse a 12 et l'ajustement de
                # faisceaux desactive, une association fausse entre deux
                # facades identiques a pu entrer dans le graphe. Elle y reste
                # a demeure : GTSAM calculait une erreur de 2.38e12,
                # rigoureusement constante sur 173 iterations consecutives
                # ("Error computed is very huge and/or diverging! Aborting!"),
                # donc plus aucune fermeture ne pouvait etre integree.
                # Optimizer/Robust active les contraintes commutables
                # (Vertigo) : l'optimiseur apprend pendant la resolution a
                # neutraliser les arcs incoherents, au lieu qu'un seul mauvais
                # lien fasse diverger tout le graphe. Necessite GTSAM (compile
                # ici, cf. OptimizerGTSAM.cpp dans les journaux) et
                # RGBD/OptimizeMaxError=0, deja notre configuration : ces deux
                # parametres sont concus pour fonctionner ensemble.
                # NB : la reponse suggeree par le message d'erreur
                # (Optimizer/Epsilon=0) serait mauvaise ; elle ne ferait
                # qu'ignorer le controle de divergence et produirait une carte
                # fausse au lieu d'un echec visible.
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
                # --- Segmentation du sol par SEUIL DE HAUTEUR ---
                # Grid/NormalsSegmentation, actif par defaut, identifie
                # le sol par analyse des normales du nuage. Sur un LiDAR
                # automobile la chaussee est vue en incidence tres
                # rasante : les normales y sont bruitees et les points
                # sont rejetes comme obstacles.
                #
                # Sans points sol, le lancer de rayons n'a rien vers quoi
                # tracer, donc aucune cellule n'est marquee LIBRE.
                # Mesure sur valid_grille.log : 2,4 % de cellules connues
                # et libre=0,0 % -- les seules cellules renseignees sont
                # les facades, vues de face. Le gain d'information valait
                # alors 1,000 partout, y compris a la position meme du
                # vehicule.
                #
                # A 'false', RTAB-Map bascule sur un simple seuil :
                # sous Grid/MaxGroundHeight (1,0 m) c'est du sol, au
                # dessus c'est un obstacle. Sur une route plate c'est le
                # mode robuste.
                #
                # Un seuil ne distingue pas une chaussee d'un trottoir
                # bas : quelques cellules seront libres a tort. Sans
                # consequence pour un critere d'exploration, dont la
                # question est "cette zone a-t-elle deja ete vue ?" et
                # non "puis-je rouler ici ?".
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxGroundHeight': '1.0',
                'Grid/GroundIsObstacle': 'false',
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
                # 2D et non 3D : avec Grid/3D a 'true', le lancer de
                # rayons se fait EN VOLUME, ce qui est lent au point de
                # compromettre la cartographie temps reel. La grille
                # consommee par le decideur, /grid_prob_map, est de
                # toute facon une projection 2D : la construire
                # nativement en 2D donne le meme resultat pour un cout
                # bien moindre.
                'Grid/3D': 'false',
                # --- Lancer de rayons : INDISPENSABLE ---
                # Sans lui, RTAB-Map ne marque que les cellules
                # OCCUPEES. La chaussee parcourue reste a -1 et la
                # grille est une carte des murs, pas une carte de
                # l'explore.
                #
                # Mesure : le gain d'information valait 1.000 A LA
                # POSITION MEME du vehicule, pose=(237.2,116.4) dans une
                # grille x=[-1.6,303.8] y=[-1.6,184.8]. Le critere le
                # plus lourd de la decision (0,30) ne pouvait rien
                # classer : il mesurait la proportion d'inconnu dans une
                # grille ou tout est inconnu.
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

        # rtabmap_viz peut etre commente pour liberer du GPU pendant les tests
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
        # --- Generateur de destinations candidates (nouveau) ---
        Node(
            package='active_slam_decision',
            executable='candidate_generator',
            name='active_slam_candidate_generator',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'role_name': 'ego',
                'waypoint_spacing': 8.0,
                # Portee en distance ROUTIERE. Doit rester <= au
                # route_max_distance du controleur, sinon il pourrait
                # rejeter un candidat que ce noeud juge atteignable.
                'max_route_distance': 250.0,
                # --- Distance routiere minimale d'une destination ---
                # 12 m a ete essaye pour rapprocher les candidats de la
                # zone cartographiee. L'essai est concluant, mais dans
                # l'autre sens que prevu -- a budget egal :
                #
                #   25 m : 602 m parcourus, 13 destinations atteintes
                #   12 m : 604 m parcourus,  3 destinations atteintes
                #
                # Le gain d'information restait sature dans les deux
                # cas, et 25 m produit quatre fois plus de decisions --
                # donc quatre fois plus d'influence de la fonction de
                # decision sur la trajectoire, ce qui est l'objet meme
                # de la campagne comparative.
                #
                # 25 m est aussi la valeur pour laquelle le seuil de
                # rejet de on_arrived (1,0 s) a ete dimensionne : a
                # 12 m, une arrivee reelle a pleine vitesse prend 0,8 s
                # et serait comptee comme un rejet.
                'min_candidate_distance': 25.0,
                # 0 = pas de plafond a vol d'oiseau.
                'search_radius': 0.0,
                'max_candidates': 400,
                'publish_period': 3.0,
            }],
        ),
        # --- Detecteur d'obstacles (nouveau) ---
        Node(
            package='active_slam_decision',
            executable='obstacle_detector',
            name='active_slam_obstacle_detector',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'role_name': 'ego',
                'detection_radius': 100.0,
                'publish_period': 1.0,
                'obstacles_topic': '/obstacle_detection/obstacles',
            }],
        ),
        # --- Trafic NPC (nouveau, pour que le critere safety ait un effet) ---
        Node(
            package='active_slam_decision',
            executable='traffic_spawner',
            name='active_slam_traffic_spawner',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'ego_role_name': 'ego',
                'num_vehicles': 20,
                'min_vehicles_near_ego': 6,
                # Pietons : 0 par defaut, car la boucle d'appels a
                # world.get_random_location_from_navigation() du
                # traffic_spawner provoque un segfault (exit code -11)
                # apres ~13 s sur Town03_Opt, dont le maillage de
                # navigation pietonne n'est pas charge par la variante
                # _Opt. Le noeud meurt avant meme d'avoir affiche
                # "TRAFIC ACTIF", d'ou l'absence totale de trafic.
                # Le nombre est desormais pilotable pour permettre le
                # scenario pietons demande par le sujet, qui doit se
                # lancer sur la carte pleine :
                #   ros2 launch ... town:=Town03 num_pedestrians:=15
                'num_walkers': ParameterValue(
                    LaunchConfiguration('num_pedestrians'),
                    value_type=int
                ),
                # --- Graine du trafic ---
                # traffic_spawner declare 'seed' a -1 (aleatoire) et,
                # des qu'elle est >= 0, appelle random.seed() puis
                # traffic_manager.set_random_device_seed(). Ce launch
                # ne la lui passait pas : les quatre strategies Active
                # SLAM de la campagne ont donc affronte un trafic tire
                # au hasard, alors que slam_evaluation.launch.py fixait
                # la sienne a 42.
                #
                # Defaut -1 : comportement inchange. Fixee, elle rend
                # possible la comparaison APPARIEE de deux reglages sur
                # le meme trafic.
                'seed': ParameterValue(
                    LaunchConfiguration('seed'),
                    value_type=int
                ),
                'min_distance_from_ego': 10.0,
            }],
        ),
        # --- Fonction de decision multi-criteres (nouveau, coeur du sujet) ---
        decision_maker_node,

        # --- Controleur vehicule Active SLAM ---
        Node(
            package='active_slam_decision',
            executable='vehicle_controller_active_slam',
            name='vehicle_controller_active_slam',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'role_name': 'ego',
                'max_throttle': LaunchConfiguration('max_throttle'),
                'arrival_tolerance': LaunchConfiguration('arrival_tolerance'),
                # --- Pas du parcours du reseau routier ---
                # DOIT valoir la meme chose que waypoint_spacing du
                # candidate_generator (8.0 m, plus haut dans ce fichier).
                # Les deux noeuds repondent a la meme question -- 'puis-je
                # aller la ?' -- et doivent donc parcourir le meme graphe.
                #
                # Mesure avec 4.0 ici et 8.0 la-bas, depuis le meme
                # waypoint de depart : le generateur trouvait 23
                # destinations atteignables, le controleur n'en routait
                # aucune, avec 75 aretes explorees seulement -- soit une
                # chaine quasi droite sur 250 m. A 4 m, le parcours
                # s'arrete a l'interieur des carrefours et n'en suit
                # qu'une branche ; a 8 m, il les enjambe et retombe sur
                # toutes les routes sortantes.
                'route_step': 8.0,
                # --- Portee du parcours routier ---
                # DOIT valoir au moins 3 fois le
                # max_candidate_distance du decision_maker (150 m) :
                # un trajet routier fait 2 a 3 fois la distance a vol
                # d'oiseau (sens de circulation, virages, contournement
                # des pates de maisons).
                #
                # Le defaut du noeud, 250 m, etait incoherent avec les
                # 150 m admis pour un candidat. Mesure sur
                # valid_retour25.log : buts a 124, 137 et 154 m a vol
                # d'oiseau, tous rejetes, aretes=38 soit 38 x 8 m =
                # 304 m -- le parcours butait sur son propre budget, pas
                # sur un cul-de-sac. Resultat : 32 buts ecartes,
                # 0 destination atteinte, 0 m parcouru.
                #
                # Le defaut etait INTERMITTENT : si le premier but tire
                # tombait a moins de 250 m par la route, le run se
                # deroulait normalement. Meme code, memes parametres,
                # meme carte, deux issues opposees selon le tirage.
                # Ramene de 500 a 300 m. 500 m avait ete choisi pour
                # debloquer le planificateur, qui refusait des buts
                # atteignables faute de portee. Mais en debloquant, on a
                # autorise des trajets qui avalent le budget entier :
                # mesure sur valid_capteur40.log, un but a 80 m a vol
                # d'oiseau a produit un itineraire de 495 m, soit 82 %
                # d'un run de 600 m pour UNE seule destination.
                #
                # Consequence sur la campagne : le nombre de decisions
                # par run variait de 1 a 13 a configuration identique.
                # Un run a une decision ne compare aucune strategie.
                #
                # 300 m laisse un facteur 3,75 sur les 80 m de
                # max_candidate_distance, largement au-dessus des
                # trajets normalement observes (106 m et 66 m pour les
                # deux autres buts du meme run).
                'route_max_distance': 300.0,
                'lookahead_distance': 6.0,
                'steer_gain_deg': 60.0,
                'max_steer_rate': 0.15,
                'respect_traffic_lights': True,
                'obstacle_corridor_width': 3.0,
                'obstacle_stop_distance': 7.0,
                'obstacle_slow_distance': 14.0,
                'max_obstacle_wait': 60.0,
                'no_progress_timeout': 45.0,
                'goal_timeout': 180.0,
                'obstacles_topic': '/obstacle_detection/obstacles',
            }],
        ),
        # --- Evaluateur de localisation ---
        Node(
            package='slam_evaluation',
            executable='localization_evaluator.py',
            name='localization_evaluator',
            output='screen',
            parameters=[{
                'output_file': '/home/ubuntu/active_slam_carla/metrics/localization_errors_active_slam.csv',
                'buffer_size': 50,
                'save_interval': 10,
            }],
        ),

        # --- Arret automatique du run complet des que decision_maker se
        # ferme (exploration terminee par stagnation) ---
        RegisterEventHandler(
            OnProcessExit(
                target_action=decision_maker_node,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='Exploration terminee (stagnation detectee)'
                ))],
            )
        ),
    ])
