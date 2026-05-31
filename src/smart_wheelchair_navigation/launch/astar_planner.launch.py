import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    managed_node_args = ['--ros-args', '--log-level', log_level]
    lifecycle_args = ['--ros-args', '--log-level', log_level]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nodes = GroupAction([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            arguments=managed_node_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            arguments=managed_node_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_astar',
            output='screen',
            arguments=lifecycle_args,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 20.0,
                'node_names': ['map_server', 'planner_server'],
            }],
        ),
    ])

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav, 'maps', 'hospital_map.yaml'),
            description='Full path to map file to load.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate the A* planner demo.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level.',
        ),
        nodes,
    ])
