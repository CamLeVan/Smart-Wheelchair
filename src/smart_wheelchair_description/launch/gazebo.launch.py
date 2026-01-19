import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'smart_wheelchair_description'
    urdf_file_name = 'wheelchair.urdf'
    world_file_name = 'hospital.world'
    
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)
        
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        world_file_name)

    # Read URDF content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Start Gazebo with World
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),
        
        # Spawn Entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'smart_wheelchair', '-topic', 'robot_description', '-z', '0.1'],
            output='screen'),
            
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]),
    ])
