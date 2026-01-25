import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')
   
    # Define arguments
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav, 'maps', 'hospital_map.yaml'),
        description='Full path to map file to load'
    )
   
    nav2_params_path = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')


    return LaunchDescription([
        map_file_arg,
       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'params_file': nav2_params_path,
                'autostart': 'true',  # QUAN TRỌNG: Tự động kích hoạt
                'use_composition': 'True'
            }.items()
        )
    ])





