import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    uvs_mapserver_dir = get_package_share_directory('uvs_mapserver')
    config = os.path.join(uvs_mapserver_dir, 'config', 'mapserver_config.yaml')
    
    return LaunchDescription([
        Node(
            package='uvs_mapserver',
            executable='uvs_mapserver',
            name='uvs_mapserver',
            output='screen',
            parameters=[config]
        )
    ])
    