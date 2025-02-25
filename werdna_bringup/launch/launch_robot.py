import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
def generate_launch_description():


    # Launch teleop, MPU6050 driver, odometry, agent and robot hardware interface & controllers

    description_prefix = get_package_share_directory("werdna_description")
    teleop_prefix = get_package_share_directory("werdna_teleop")
    agent_prefix = get_package_share_directory("werdna_agent")
    imu_prefix = get_package_share_directory("ros2_mpu6050_")

    teleop_launch_file = os.path.join(teleop_prefix, "launch", "werdna_teleop.launch.py")

    ros2_control_launch_file = os.path.join(description_prefix, "launch", "ros2_control.launch.py")

    imu_launch_file = os.path.join(imu_prefix, "launch", "ros2_mpu6050.launch.py")

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file),
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_control_launch_file)
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file)
    )
    
    agent_node = Node(
        package=agent_prefix,
        executable="werdna_agent_node",
    )
    
    return LaunchDescription([
        teleop,
        ros2_control,
        # odometry_node,
        agent_node,
        imu,
    ])