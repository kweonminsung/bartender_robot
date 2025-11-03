from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bartender_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'bartender_robot.xacro')

    # generate robot_description from xacro at runtime
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # start joint_state_publisher (provides joint_states for robot model)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # start robot_state_publisher so TF is available
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # start rviz and load the included rviz config if present
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    rviz_args = []
    if os.path.exists(rviz_config):
        rviz_args = ['-d', rviz_config]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz,
    ])
