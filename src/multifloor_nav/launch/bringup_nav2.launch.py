from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])),
            launch_arguments={
                'map': 'maps/robocon_level_ground.yaml',
                'params_file': 'config/nav2_params.yaml',  # first_map_only: false in here
                'use_sim_time': 'true',
            }.items(),
        ),
    ])