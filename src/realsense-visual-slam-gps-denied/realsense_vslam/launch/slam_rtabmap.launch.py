import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Tüm düğümlerde ortak kullanılacak kamera ve zaman parametreleri
    base_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'approx_sync': True,
        'qos_image': 2,
        'qos_camera_info': 2,
        'qos_depth': 2,
        'qos_odom': 2
    }
    
    # Haritacıya özel SLAM parametreleri
    slam_params = base_params.copy()
    slam_params.update({
        'subscribe_odom_info': False,
        'Grid/3D': 'true',
        'Grid/MaxObstacleHeight': '2.5',
        'RGBD/ProximityBySpace': 'true',
        
        'Kp/DetectorStrategy': '2',
        'Odom/Strategy': '1',       # 1=Optical Flow kullan (Pürüzsüz ortamlarda daha iyi)
        'Vis/CorType': '1',         # Optik akış eşleşmesi
        'Odom/ResetCountdown': '1'  # Kaybolursa anında resetle
        
    })

    # Kamera Topic'leri
    remappings = [
        ('rgb/image', '/master/camera/image_raw'),
        ('rgb/camera_info', '/master/camera/camera_info'),
        ('depth/image', '/master/camera/depth/image_raw')
    ]

    return LaunchDescription([
        # 1. Boyun Kemiği (Kamera Konumu)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=['0.22', '0.0', '0.0', '-1.5708', '0.0', '-1.5708', 'base_link', 'realsense_link']
        ),
        
        # 2. YENİ EKLENEN BEYİN: GÖRSEL ODOMETRİ (VO) MOTORU!
        # GPS kesildiğinde Hybrid_Localizer bu veriyi kullanacak.
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=[{**base_params, 'publish_tf': False, 'odom_frame_id': 'odom'}],
            remappings=remappings + [('odom', '/rtabmap/vo')]
        ),
        
        # 3. HARİTACI (RTAB-Map)
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[slam_params],
            remappings=remappings + [('odom', '/master/odom')],
            arguments=['-d']
        ),
        
        # 4. GÖRSELLEŞTİRİCİ
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=[slam_params],
            remappings=remappings + [('odom', '/master/odom')]
        )
    ])