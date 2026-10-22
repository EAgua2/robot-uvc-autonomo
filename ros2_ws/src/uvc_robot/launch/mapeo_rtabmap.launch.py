"""
Mapeo con Kinect v1 + RTAB-Map (SLAM visual RGB-D).

Lanza:
  1. Driver del Kinect v1 (kinect_ros2)
  2. TF estática base_link -> kinect_rgb
  3. Odometría visual (rtabmap_odom/rgbd_odometry) -> publica odom -> base_link
  4. RTAB-Map SLAM (rtabmap_slam/rtabmap)        -> publica map -> odom
  5. Guardado automático del mapa (auto_map_saver_pkg)

Uso:
  ros2 launch uvc_robot mapeo_rtabmap.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Tópicos del Kinect (iguales para odometría visual y SLAM)
    kinect_remappings = [
        ('rgb/image', '/rgb/image_raw'),
        ('depth/image', '/depth/image_raw'),
        ('rgb/camera_info', '/rgb/camera_info'),
        ('depth/camera_info', '/depth/camera_info'),
    ]

    # Parámetros comunes para RTAB-Map y odometría visual
    common_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.5,
        'queue_size': 30,
        'sync_queue_size': 30,
        'publish_tf': True,
        'Rtabmap/DetectionRate': '1',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'cloud_noise_filtering_radius': '0.05',
        'cloud_noise_filtering_min_neighbors': '5',
    }

    # Parámetros específicos de RTAB-Map SLAM
    rtabmap_params = common_params.copy()
    rtabmap_params.update({
        'Mem/NotLinkedNodesKept': 'false',
        'RGBD/AngularUpdate': '0.05',   # rad: agrega un nodo al mapa tras girar ~3°
        'RGBD/LinearUpdate': '0.20',    # m:   o tras avanzar 20 cm
    })

    return LaunchDescription([

        # Driver del Kinect v1
        Node(
            package='kinect_ros2',
            executable='kinect_ros2_node',
            name='kinect1',
            parameters=[{'depth_registration': True}],
            output='screen',
        ),

        # TF estática base_link -> kinect_rgb
        # El Kinect está a 35 cm de altura; roll/yaw de -90° convierten
        # el marco óptico (z hacia adelante) al marco del robot (x hacia adelante).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--frame-id', 'base_link',
                '--child-frame-id', 'kinect_rgb',
                '--x', '0',
                '--y', '0',
                '--z', '0.35',
                '--roll', '-1.5708',
                '--pitch', '0',
                '--yaw', '-1.5708',
            ],
            output='screen',
        ),

        # Odometría visual RGB-D
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[common_params],
            remappings=kinect_remappings,
            output='screen',
        ),

        # RTAB-Map SLAM
        # '-d' borra la base de datos (~/.ros/rtabmap.db) al iniciar: cada
        # ejecución crea un mapa nuevo. Quítalo para continuar un mapa existente.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[rtabmap_params],
            remappings=kinect_remappings,
            arguments=['-d'],
            output='screen',
        ),

        # Guardado automático del mapa
        Node(
            package='auto_map_saver_pkg',
            executable='auto_map_saver',
            name='auto_map_saver',
            output='screen',
        ),
    ])
