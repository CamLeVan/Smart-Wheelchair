import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _mode_condition(mode_name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == '", mode_name, "'"]))


def _rviz_condition():
    return IfCondition(LaunchConfiguration('use_rviz'))


def generate_launch_description():
    pkg_nav = get_package_share_directory('smart_wheelchair_navigation')
    pkg_description = get_package_share_directory('smart_wheelchair_description')

    map_path = os.path.join(pkg_nav, 'maps', 'hospital_map.yaml')
    nav_params_path = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_nav, 'config', 'nav_config.rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')
        ),
        condition=_mode_condition('slam')
    )

    astar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'astar_planner.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
        condition=_mode_condition('nav')
    )

    nav_full_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
        condition=_mode_condition('nav_full')
    )

    human_tracker = Node(
        package='smart_wheelchair_vision',
        executable='human_tracker',
        name='human_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            'model_path': LaunchConfiguration('model_path'),
            'camera_fov_rad': LaunchConfiguration('camera_fov_rad'),
            'target_distance': LaunchConfiguration('target_distance'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'show_debug_view': ParameterValue(LaunchConfiguration('show_debug_view'), value_type=bool),
            'angular_deadband_px': LaunchConfiguration('angular_deadband_px'),
            'target_bbox_width_ratio': LaunchConfiguration('target_bbox_width_ratio'),
            'vision_backend': LaunchConfiguration('vision_backend'),
            'sim_target_start_x': LaunchConfiguration('sim_target_start_x'),
            'sim_target_end_x': LaunchConfiguration('sim_target_end_x'),
            'sim_target_y': LaunchConfiguration('sim_target_y'),
            'sim_target_period': LaunchConfiguration('sim_target_period'),
        }],
        condition=_mode_condition('follow')
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=_rviz_condition()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='world',
            description='Gazebo demo mode: world, slam, nav, nav_full, or follow. Run one control mode at a time.'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Map YAML used by Nav2 in mode:=nav.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav_params_path,
            description='Nav2 parameters used in mode:=nav.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz with the project demo config.'
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Open Gazebo GUI. Set false to run gzserver only.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='RViz config file used when use_rviz:=true.'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt',
            description='YOLO model path used in mode:=follow.'
        ),
        DeclareLaunchArgument(
            'camera_fov_rad',
            default_value='1.085',
            description='Horizontal camera FOV in radians used by Lidar fusion.'
        ),
        DeclareLaunchArgument(
            'target_distance',
            default_value='1.2',
            description='Follow-me target distance in meters.'
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.5',
            description='Follow-me maximum linear speed in m/s.'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='1.0',
            description='Follow-me maximum angular speed in rad/s.'
        ),
        DeclareLaunchArgument(
            'show_debug_view',
            default_value='false',
            description='Open the OpenCV follow-me debug camera window.'
        ),
        DeclareLaunchArgument(
            'angular_deadband_px',
            default_value='40.0',
            description='Pixel deadband before follow-me turns toward the target.'
        ),
        DeclareLaunchArgument(
            'target_bbox_width_ratio',
            default_value='0.45',
            description='Fallback visual target width ratio when Lidar has no actor return.'
        ),
        DeclareLaunchArgument(
            'vision_backend',
            default_value='sim_scan',
            description='Follow-me detector backend: sim_scan, opencv_hog, or yolo.'
        ),
        DeclareLaunchArgument(
            'sim_target_start_x',
            default_value='2.2',
            description='Gazebo actor start x used by the stable sim follow backend.'
        ),
        DeclareLaunchArgument(
            'sim_target_end_x',
            default_value='3.4',
            description='Gazebo actor end x used by the stable sim follow backend.'
        ),
        DeclareLaunchArgument(
            'sim_target_y',
            default_value='0.0',
            description='Gazebo actor y used by the stable sim follow backend.'
        ),
        DeclareLaunchArgument(
            'sim_target_period',
            default_value='20.0',
            description='Gazebo actor trajectory period used by the stable sim follow backend.'
        ),
        gazebo_launch,
        slam_launch,
        astar_launch,
        nav_full_launch,
        human_tracker,
        rviz,
    ])
