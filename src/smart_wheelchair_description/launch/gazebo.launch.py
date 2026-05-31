import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'smart_wheelchair_description'
    urdf_file_name = 'wheelchair.urdf'
    world_file_name = 'hospital.world'
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name,
    )

    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        world_file_name,
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    gzserver = [
        'gzserver',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        world_path,
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Open Gazebo GUI. Set false to run gzserver only.',
        ),
        ExecuteProcess(
            cmd=gzserver,
            output='screen',
            condition=IfCondition(gazebo_gui),
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(gazebo_gui),
        ),
        ExecuteProcess(
            cmd=gzserver,
            output='screen',
            condition=UnlessCondition(gazebo_gui),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'smart_wheelchair',
                '-topic', 'robot_description',
                '-z', '0.3',
                '-timeout', '120.0',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
    ])
