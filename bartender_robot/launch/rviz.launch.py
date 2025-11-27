import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Set false if using real robot
USE_JOINT_STATE_PUBLISHER = True

def generate_launch_description():
    pkg_path = get_package_share_directory('bartender_robot')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'bartender_robot.urdf')

    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()

    # Start joint_state_publisher (provides joint_states for robot model)
    if USE_JOINT_STATE_PUBLISHER:
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )

    # Start robot_state_publisher so TF is available
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    print(f'pkg_path: {pkg_path}')
    rviz_config = os.path.join(pkg_path, 'rviz', 'robot.rviz')
    
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

    nodes = [robot_state_publisher, rviz]
    if USE_JOINT_STATE_PUBLISHER:
        nodes.insert(0, joint_state_publisher)
    
    return LaunchDescription(nodes)
