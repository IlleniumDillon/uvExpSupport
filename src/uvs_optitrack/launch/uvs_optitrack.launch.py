import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    uvs_optitrack_dir = get_package_share_directory('uvs_optitrack')
    config = os.path.join(uvs_optitrack_dir, 'config', 'optitrack_parameter.yaml')
    
    return LaunchDescription([
        Node(
            package='uvs_optitrack',
            executable='uvs_optitrack',
            name='uvs_optitrack',
            output='screen',
            parameters=[config]
        )
    ])
    