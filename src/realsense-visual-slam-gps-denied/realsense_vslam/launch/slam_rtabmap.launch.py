import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':True,
          'queue_size': 20,
          'Grid/3D': 'true',
          'Grid/MaxObstacleHeight': '2.5',
          'RGBD/ProximityBySpace': 'true',
          'Kp/DetectorStrategy': '2',
          
          # 1. ÇÖZÜM: QoS İNADINI KIRMAK (2 = Best Effort)
          'qos_image': 2,
          'qos_camera_info': 2,
          'qos_depth': 2,
          'qos_odom': 2
    }]

    remappings=[
          ('rgb/image', '/master/camera/image_raw'),
          ('rgb/camera_info', '/master/camera/camera_info'),
          ('depth/image', '/master/camera/depth/image_raw'),
          ('odom', '/master/odom')
    ]

    return LaunchDescription([
        
        # 2. ÇÖZÜM: EKSİK KEMİĞİ EKLEME (Static TF Broadcaster)
        # Kameranın gövdenin 22 cm önünde olduğunu ve optik açısını sisteme söylüyoruz.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                '--x', '0.22', '--y', '0.0', '--z', '0.0', 
                '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708', 
                '--frame-id', 'base_link', '--child-frame-id', 'realsense_link'
            ]
        ),
        
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']
        ),
        
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings
        )
    ])