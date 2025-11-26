#include "motion_planning_node.hpp"

MotionPlanningNode::MotionPlanningNode(const rclcpp::NodeOptions & options)
: Node("motion_planning_node", options)
{
  RCLCPP_INFO(get_logger(), "Motion planning node has been started.");

  // Setup MoveGroupInterface
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "bartender_robot");

  // Setup PlanningSceneInterface
  planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Setup MoveItVisualTools
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    shared_from_this(), "base_link", "moveit_visual_markers", move_group_interface_->getRobotModel());
  
  visual_tools_->deleteAllMarkers();
  visual_tools_->loadRemoteControl();

  RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
  RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());
}
