import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'approx_sync': True
    }]

    remappings=[
        ('rgb/image', '/master/camera/image_raw'),
        ('rgb/camera_info', '/master/camera/camera_info'),
        ('depth/image', '/master/camera/depth/image_raw'),
        ('odom', '/master/odom')
    ]

    return LaunchDescription([
        # İŞTE ÇÖZÜM: Kameranın dronun gövdesine göre konumunu yayınlayan düğüm
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_tf',
            # Argümanlar: x, y, z, yaw, pitch, roll, parent_frame, child_frame
            # Not: Kameranın lens açısını düzeltmek için -90 derecelik (-1.5708 radyan) optik düzeltmeler ekledik.
            arguments=['0.2', '0.0', '0.0', '-1.5708', '0.0', '-1.5708', 'base_link', 'realsense_link']
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