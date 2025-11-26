#ifndef PLAN_PLAYER_HPP
#define PLAN_PLAYER_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>

class PlanPlayer : public rclcpp::Node
{
public:
    PlanPlayer();
    void loadAndPublishPlan(const std::string& csv_path);

private:
    struct TrajectoryPoint {
        double time;
        std::vector<double> positions;
    };

    std::vector<TrajectoryPoint> parseCsvFile(const std::string& csv_path);
    void publishTrajectory(const std::vector<TrajectoryPoint>& points);
    void playTrajectoryAsJointPositions(const std::vector<TrajectoryPoint>& points);

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
    std::vector<std::string> joint_names_;
};

#endif // PLAN_PLAYER_HPP
