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
    
    ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package='uvs_embmaster',
            executable='uvs_embmaster',
            output='screen'
        )
    )
    
    ld.add_action(
        Node(
            package='uvs_manual',
            executable='uvs_manual_keyboard',
            output='screen'
        )
    )
    
    return ld