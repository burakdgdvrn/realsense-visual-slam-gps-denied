import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Gazebo saatini kullan!
    sim_time = {'use_sim_time': True}
    
    return LaunchDescription([
        Node(
            package='realsense_vslam',
            executable='formation_controller.py',
            name='formation_controller',
            output='screen',
            parameters=[sim_time]
        ),
        Node(
            package='realsense_vslam',
            executable='kinematic_physics.py',
            name='kinematic_physics',
            output='screen',
            parameters=[sim_time]
        ),
        Node(
            package='realsense_vslam',
            executable='odom_broadcaster.py',
            name='odom_broadcaster',
            output='screen',
            parameters=[sim_time]
        )
    ])