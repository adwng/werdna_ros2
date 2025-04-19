import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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


    # agent_node = ExecuteProcess(
    #     cmd=["ros2", "run", "werdna_agent", "werdna_agent_node"],
    #     output="screen",
    # )

    agent_node = ExecuteProcess(
        cmd=["ros2", "run", "werdna_pid", "werdna_pid_node"],
        output="screen",
    )

    rosboard = ExecuteProcess(
        cmd=["ros2", "run", "rosboard", "rosboard_node"],
        output="screen"
    )
    
    return LaunchDescription([
        teleop,
        ros2_control,
        agent_node,
        rosboard,
    ])