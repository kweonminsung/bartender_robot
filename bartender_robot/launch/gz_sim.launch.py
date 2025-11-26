import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    pkg_path = get_package_share_directory('bartender_robot')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'bartender_robot.urdf')

    robot_description = Command(['xacro ', urdf_file_path])

    # Add real Gazebo resource path(installation directory)
    install_dir = os.path.dirname(pkg_path)
    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )

    # Start robot_state_publisher so TF is available
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
    )

    # Spawn entity in running Gazebo (assumes Gazebo is already running)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bartender_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Bridge for clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
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

    return LaunchDescription([
        gz_resource_path,
        gz,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        bridge,
        robot_state_publisher,
        spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])

