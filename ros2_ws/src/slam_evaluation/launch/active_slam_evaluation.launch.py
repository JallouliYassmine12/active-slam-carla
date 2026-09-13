from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import datetime


def generate_launch_description():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # Vehicle Controller — Active SLAM (pas de waypoints, attend /goal + /decision_maker/maneuver)
    # NB : ce noeud ignore carla_host/carla_port (il connecte en dur sur localhost:2000),
    # donc ces deux arguments ne servent ici qu'a bridge_node.
    controller_node = Node(
        package='vehicle_controller',
        executable='vehicle_controller_active_slam',
        name='vehicle_controller_active_slam',
        output='screen',
        parameters=[{
            'max_throttle': LaunchConfiguration('max_throttle'),
            'arrival_tolerance': LaunchConfiguration('arrival_tolerance'),
        }],
    )

    # Declenche l'arret global (rtabmap inclus) des que le controleur se termine
    shutdown_on_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Trajet Active SLAM termine par le controleur'))]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('carla_host', default_value='192.168.1.6'),
        DeclareLaunchArgument('carla_port', default_value='2000'),
        DeclareLaunchArgument('max_throttle', default_value='0.5'),
        DeclareLaunchArgument('arrival_tolerance', default_value='2.0'),

        # CARLA Sensors Bridge (identique au SLAM classique — spawn le vehicule ego)
        Node(
            package='carla_sensors_bridge',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('carla_host'),
                'port': LaunchConfiguration('carla_port'),
                'town': 'Town01',
                'role_name': 'ego',
                'synchronous_mode': True,
                'fixed_delta_seconds': 0.05,
            }],
        ),

        # Static transforms
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

        # Filtre IMU (orientation a partir du gyro + accelerometre)
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

        # Odometrie LiDAR (ICP)
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
                'Icp/MaxTranslation': '2.0',
                'Icp/MaxRotation': '1.0',
                'Icp/MaxCorrespondenceDistance': '1.0',
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

        # RTAB-Map : cartographie (camera + lidar), position venant de l'ICP
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
                'database_path': f'/home/ubuntu/active_slam_carla/maps/active_slam_eval_{timestamp}.db',
                'Mem/IncrementalMemory': 'true',
                'Rtabmap/DetectionRate': '1.0',
                'Reg/Force3DoF': 'true',
                'Grid/Sensor': '2',
                'Grid/FromDepth': 'false',
                'Grid/FromScan': 'false',
                'Grid/FromPointCloud': 'true',
                'Grid/MapFrameProjection': 'true',
                'Grid/MaxObstacleHeight': '5.0',
                'Grid/MinGroundHeight': '-2.0',
                'Grid/MaxGroundHeight': '1.0',
                'Grid/GroundIsObstacle': 'false',
                'Grid/RangeMax': '50.0',
                'Grid/CellSize': '0.05',
                'Grid/3D': 'true',
                'Grid/RayTracing': 'true',
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

        # RTAB-Map Visualization
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

        # Vehicle Controller Active SLAM (defini plus haut)
        controller_node,

        # Localization Evaluator (fichier de sortie distinct du SLAM classique)
        Node(
            package='slam_evaluation',
            executable='localization_evaluator.py',
            name='localization_evaluator',
            output='screen',
            parameters=[{
                'output_file': '/home/ubuntu/active_slam_carla/metrics/active_slam_localization_errors.csv',
                'buffer_size': 50,
                'save_interval': 10,
            }],
        ),

        # Arret global des que le controleur se termine
        shutdown_on_controller_exit,
    ])
