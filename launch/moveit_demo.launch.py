#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF directly (not a xacro file)
    robot_description_content = Command(
        [
            "cat ",
            PathJoinSubstitution(
                [FindPackageShare("bartender_robot"), "urdf", "bartender_robot.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get SRDF
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                "cat ",
                PathJoinSubstitution(
                    [FindPackageShare("bartender_robot"), "config", "moveit", "bartender_robot.srdf"]
                ),
            ]
        ),
        value_type=str
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics config
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("bartender_robot"), "config", "moveit", "kinematics.yaml"]
    )

    # Planning functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = PathJoinSubstitution(
        [FindPackageShare("bartender_robot"), "config", "moveit", "ompl_planning.yaml"]
    )

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("bartender_robot"), "config", "moveit", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Joint limits
    joint_limits_yaml = PathJoinSubstitution(
        [FindPackageShare("bartender_robot"), "config", "moveit", "joint_limits.yaml"]
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            ompl_planning_yaml,
            trajectory_execution,
            moveit_simple_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bartender_robot"), "rviz", "robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF for virtual joint
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", 
                   "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
                   "--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ros2_control using mock hardware for MoveIt demo
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("bartender_robot"), "config", "moveit", "ros2_controllers.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        static_tf_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
