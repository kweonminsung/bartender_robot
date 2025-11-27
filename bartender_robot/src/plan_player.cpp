#include "plan_player.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

PlanPlayer::PlanPlayer() : Node("plan_player")
{
    // Joint names matching the CSV header
    joint_names_ = {
        "XL_430_base_Revolute-29",
        "XL_430_shoulder_Revolute-30",
        "XL_430_upper_Revolute-31",
        "XL_430_lower_Revolute-32"
    };

    // Create publisher for joint trajectory
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    
    // Create publisher for joint positions (for joint_state_publisher node)
    joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_positions", 10);

    RCLCPP_INFO(this->get_logger(), "Plan Player Node initialized");
}

std::vector<PlanPlayer::TrajectoryPoint> PlanPlayer::parseCsvFile(const std::string& csv_path)
{
    std::vector<TrajectoryPoint> points;
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
        return points;
    }

    std::string line;
    // Skip header line
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        TrajectoryPoint point;
        
        // Read time
        std::getline(ss, value, ',');
        point.time = std::stod(value);
        
        // Read joint positions
        while (std::getline(ss, value, ',')) {
            point.positions.push_back(std::stod(value));
        }
        
        if (point.positions.size() == joint_names_.size()) {
            points.push_back(point);
        }
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points from CSV", points.size());
    return points;
}

void PlanPlayer::publishTrajectory(const std::vector<TrajectoryPoint>& points)
{
    if (points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No trajectory points to publish");
        return;
    }

    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.header.stamp = this->now();
    trajectory_msg.joint_names = joint_names_;

    // Convert trajectory points to ROS message
    for (const auto& point : points) {
        trajectory_msgs::msg::JointTrajectoryPoint traj_point;
        traj_point.positions = point.positions;
        traj_point.time_from_start = rclcpp::Duration::from_seconds(point.time);
        trajectory_msg.points.push_back(traj_point);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing trajectory with %zu points", trajectory_msg.points.size());
    trajectory_pub_->publish(trajectory_msg);
}

void PlanPlayer::loadAndPublishPlan(const std::string& csv_path)
{
    RCLCPP_INFO(this->get_logger(), "Loading plan from: %s", csv_path.c_str());
    auto points = parseCsvFile(csv_path);
    
    if (!points.empty()) {
        publishTrajectory(points);
        playTrajectoryAsJointPositions(points);
        RCLCPP_INFO(this->get_logger(), "Plan executed successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load plan from CSV");
    }
}

void PlanPlayer::playTrajectoryAsJointPositions(const std::vector<TrajectoryPoint>& points)
{
    if (points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No trajectory points to play");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Playing trajectory with %zu points", points.size());
    
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        
        // Publish joint positions
        auto joint_pos_msg = std_msgs::msg::Float64MultiArray();
        joint_pos_msg.data = point.positions;
        joint_position_pub_->publish(joint_pos_msg);

        // Publish single point trajectory for DynamixelController
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint traj_point;
        traj_point.positions = point.positions;
        traj_point.time_from_start = rclcpp::Duration::from_seconds(0);
        traj_msg.points.push_back(traj_point);
        trajectory_pub_->publish(traj_msg);
        
        // Sleep until next point
        if (i + 1 < points.size()) {
            double sleep_duration = points[i + 1].time - point.time;
            if (sleep_duration > 0) {
                auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(sleep_duration));
                rclcpp::sleep_for(sleep_ns);
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Finished playing trajectory");
}
