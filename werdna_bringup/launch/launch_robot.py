import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
def generate_launch_description():


    # Launch teleop,agent and robot hardware interface 

    description_prefix = get_package_share_directory("werdna_description")
    teleop_prefix = get_package_share_directory("werdna_teleop")
    agent_prefix = get_package_share_directory("werdna_agent")

    teleop_launch_file = os.path.join(teleop_prefix, "launch", "werdna_teleop.launch.py")

    ros2_control_launch_file = os.path.join(description_prefix, "launch", "ros2_control.launch.py")


    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file),
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_launch_file)
    )
    
    agent_node = Node(
        package=agent_prefix,
        name="werdna_agent",
        executable="werdna_agent_node",
    )
    
    return LaunchDescription([
        teleop,
        ros2_control,
        # agent_node,
    ])