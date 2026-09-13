from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
            'weight_information_gain': 0.30,
            'weight_localization_uncertainty': 0.20,
            'weight_loop_closure': 0.20,
            'weight_distance': 0.15,
            'weight_safety': 0.15,
            'uncertainty_reference': 15.0,
            'uncertainty_critical_threshold': 50.0,
            'sensor_range': 20.0,
            'max_candidate_distance': 80.0,
            'obstacles_topic': '/obstacle_detection/obstacles',
            'log_file': f'/home/ubuntu/active_slam_carla/metrics/decisions_{timestamp}.csv',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('carla_port', default_value='2000'),
        DeclareLaunchArgument('max_throttle', default_value='0.3'),
        DeclareLaunchArgument('arrival_tolerance', default_value='2.0'),
        # weighted (methode proposee) | random | distance | info_gain
        DeclareLaunchArgument('strategy', default_value='weighted'),

        # --- CARLA Sensors Bridge (identique a slam_evaluation.launch.py) ---
        Node(
            package='carla_sensors_bridge',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('carla_host'),
                'port': LaunchConfiguration('carla_port'),
                'town': 'Town03',
                'role_name': 'ego',
                'synchronous_mode': True,
                'fixed_delta_seconds': 0.05,
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
                'Reg/Force3DoF': 'true',
                'Icp/VoxelSize': '0.2',
                'Icp/MaxTranslation': '4.0',
                'Icp/MaxRotation': '1.0',
                'Icp/MaxCorrespondenceDistance': '2.0',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlane': 'true',
                'Icp/PointToPlaneK': '10',
                'Icp/CorrespondenceRatio': '0.05',
                'Icp/DownsamplingStep': '2',
                'Icp/RangeMax': '50',
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
                'Reg/Force3DoF': 'true',
                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/Sensor': '2',
                'Grid/FromDepth': 'false',
                'Grid/FromScan': 'false',
                'Grid/FromPointCloud': 'true',
                'Grid/MapFrameProjection': 'true',
                'Grid/MaxObstacleHeight': '5.0',
                'Grid/MinGroundHeight': '-2.0',
                'Grid/MaxGroundHeight': '1.0',
                'Grid/GroundIsObstacle': 'false',
                'Grid/RangeMax': '25.0',
                'Grid/CellSize': '0.15',
                'Grid/3D': 'true',
                'Grid/RayTracing': 'false',
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
                'search_radius': 60.0,
                'min_candidate_distance': 5.0,
                'waypoint_spacing': 8.0,
                'publish_period': 3.0,
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
                'max_throttle': LaunchConfiguration('max_throttle'),
                'arrival_tolerance': LaunchConfiguration('arrival_tolerance'),
                'lookahead_distance': 4.0,
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
