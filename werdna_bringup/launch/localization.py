
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    bringup_prefix = get_package_share_directory("werdna_bringup")
    
    map_dir = LaunchConfiguration(
                'map',
                default=os.path.join(bringup_prefix, 'map','map.yaml')
             )

    param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(bringup_prefix,'config',"nav2_params.yaml")
        )


    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
    ])