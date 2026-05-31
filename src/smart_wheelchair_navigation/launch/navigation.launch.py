import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bond_timeout = LaunchConfiguration('bond_timeout')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    log_level = LaunchConfiguration('log_level')

    lifecycle_localization = ['map_server', 'amcl']
    lifecycle_navigation = [
        'controller_server',
        'planner_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

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
    lifecycle_params = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'bond_timeout': bond_timeout,
        'bond_respawn_max_duration': 45.0,
    }

    nodes = GroupAction([
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
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            arguments=managed_node_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=lifecycle_args,
            parameters=[lifecycle_params, {'node_names': lifecycle_localization}],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=managed_node_args,
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
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
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                configured_params,
                {
                    'default_bt_xml_filename': ParameterValue(bt_xml_file, value_type=str),
                    'default_nav_to_pose_bt_xml': ParameterValue(bt_xml_file, value_type=str),
                    'default_nav_through_poses_bt_xml': ParameterValue(bt_xml_file, value_type=str),
                },
            ],
            arguments=managed_node_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=managed_node_args,
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=lifecycle_args,
            parameters=[lifecycle_params, {'node_names': lifecycle_navigation}],
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
            description='Automatically activate the Nav2 stack.',
        ),
        DeclareLaunchArgument(
            'bond_timeout',
            default_value='30.0',
            description='Lifecycle bond timeout in seconds for slower VM demos.',
        ),
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=os.path.join(pkg_nav, 'config', 'simple_navigate_to_pose.xml'),
            description='Behavior tree XML used by the full navigation demo.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level.',
        ),
        nodes,
    ])
