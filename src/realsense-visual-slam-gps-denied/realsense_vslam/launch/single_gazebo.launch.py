import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_vslam')
    
    # 1. Yarattığımız yeni dünyayı değişkene atıyoruz
    world_file = os.path.join(pkg_share, 'worlds', 'vslam.world')

    # Gazebo Model Path Ayarı
    model_path = os.path.join(pkg_share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path

    # 2. Gazebo'yu başlatırken bu dünyayı kullanmasını söylüyoruz
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    master_model_path = os.path.join(pkg_share, 'models', 'master_uav', 'model.sdf')

    # Sadece Master İHA'yı haritaya ekliyoruz (Slave'ler yok)
    spawn_master = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'master_uav', '-file', master_model_path, '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_master
    ])
