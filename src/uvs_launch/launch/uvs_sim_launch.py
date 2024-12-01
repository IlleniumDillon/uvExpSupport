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
        
    # LocalIPv4Addr = DeclareLaunchArgument(
    #     "LocalIPv4Addr",default_value=TextSubstitution(text='192.168.50.114')
    # )
    # ServerIPv4Addr = DeclareLaunchArgument(
    #     "ServerIPv4Addr",default_value=TextSubstitution(text='192.168.50.194')
    # )
    # WorldName = DeclareLaunchArgument(
    #     "WorldName",default_value=TextSubstitution(text='jxl3028')
    # )
    
    config = os.path.join(package_share_dir, 'config', 'launch_host.yaml')
    
    ld = LaunchDescription()
    
    # ld.add_action(LocalIPv4Addr)
    
    # ld.add_action(ServerIPv4Addr)
    
    # ld.add_action(WorldName)
    
    ld.add_action(
        Node(
            package="uvs_optitrack",
            executable="uvs_optitrack",
            output='screen',
            parameters=[config]
        )
    )
    
    ld.add_action(
        Node(
            package='uvs_mapserver',
            executable='uvs_mapserver',
            output='screen',
            parameters=[config]
        )
    )
    
    ld.add_action(
        Node(
            package='uve_mapclient',
            executable='uve_mapclient',
            output='screen'
        )
    )
    
    ld.add_action(
        Node(
            package='uve_plan',
            executable='uve_plan_hybrid_astar_test',
            output='screen'
        )
    )
    
    ld.add_action(
        ExecuteProcess(
            cmd=["rviz2", "-d", os.path.join(package_share_dir, "rviz", "default.rviz")],
            output='screen'
        )
    )
    
    return ld