import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')
    
    # Default paths
    nav2_params_path = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    # Default map path (User should change this or pass as arg if map name differs)
    default_map_path = os.path.join(pkg_nav, 'maps', 'hospital_map.yaml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map yaml file to load'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        # Nav2 Bringup (Localization + Navigation)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file')
            }.items()
        )
    ])
