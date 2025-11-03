from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bartender_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'bartender_robot.xacro')

    # generate robot_description from xacro at runtime
    robot_description = {'robot_description': Command(['xacro', ' ', xacro_file])}

    # publish robot state
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # spawn entity in running Gazebo (assumes Gazebo is already running)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bartender_robot'],
        output='screen'
    )

    return LaunchDescription([
        jsp,
        rsp,
        spawn_entity,
    ])
