import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')
    map_file = os.path.join(pkg_nav, 'maps', 'hospital_map.yaml')
    rviz_config = os.path.join(pkg_nav, 'config', 'nav_config.rviz') # Hoặc tự load rviz sau


    return LaunchDescription([
        # Chỉ chạy đúng 1 node Map Server để test
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]),


        # Node quản lý vòng đời (Bắt buộc để Map Server hoạt động)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),
       
        # Mở luôn Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'),
    ])




