import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_vslam')
    
    # 1. Gazebo'yu başlat (Sadece Master Drone)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'single_gazebo.launch.py'))
    )
    
    # 2. Fizik, Kontrol ve Odom motorlarını başlat
    engine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'flight_engine.launch.py'))
    )
    
    # 3. Haritalama (RTAB-Map) sistemini başlat
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam_rtabmap.launch.py'))
    )

    return LaunchDescription([
        gazebo,
        engine,
        slam
    ])
