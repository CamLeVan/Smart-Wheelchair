import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')
    
    # Path to SLAM toolbox default parameters (we can create a custom one later)
    # For now, we use the default provided by slam_toolbox or just the node
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': os.path.join(pkg_nav, 'config', 'mapper_params_online_async.yaml')
            }.items()
        )
    ])
