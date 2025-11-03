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

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        joint_state_pub,
        robot_state_publisher,
    ])
