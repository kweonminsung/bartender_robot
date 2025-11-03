from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bartender_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'bartender_robot.gazebo.xacro')

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

    # spawn entity in running Gazebo (assumes Gazebo is already running)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bartender_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Gazebo Launch
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
    #     ],
    # )

    return LaunchDescription([
        gz,
        joint_state_publisher,
        robot_state_publisher,
        spawn_entity,
        # bridge
    ])

