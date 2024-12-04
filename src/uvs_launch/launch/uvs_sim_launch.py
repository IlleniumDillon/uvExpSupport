import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    package_name = 'uvs_launch'
    package_share_dir = get_package_share_directory(package_name)
    
    config = os.path.join(package_share_dir, 'config', 'launch_host.yaml')
    
    gz_package_name = 'uvs_gzplugins'
    gz_package_share_dir = get_package_share_directory(gz_package_name)
    gz_world_file_name = 'uvs_gazebo.world'
    world = os.path.join(gz_package_share_dir, 'worlds', gz_world_file_name)
    
    ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package='uvs_mapserver',
            executable='uvs_mapserver',
            output='screen',
            parameters=[config]
        )
    )
    
    ld.add_action(
        ExecuteProcess(
            cmd=["rviz2", "-d", os.path.join(package_share_dir, "rviz", "default.rviz")],
            output='screen'
        )
    )
    
    ld.add_action(
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world],
            output="screen"
        )
    )
    
    return ld