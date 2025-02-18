import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    werdna_teleop_node = Node(
        package = 'werdna_teleop',
        executable='werdna_teleop_joy_node'
    )

    return LaunchDescription(
        [joy_node, werdna_teleop_node]
    )