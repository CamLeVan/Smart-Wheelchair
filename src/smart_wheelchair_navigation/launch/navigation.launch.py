import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')
   
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav, 'maps', 'hospital_map.yaml'),
        description='Full path to map file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )


    return LaunchDescription([
        map_file_arg,
        params_file_arg,
       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',  
                'use_composition': 'False'
            }.items()
        )
    ])





