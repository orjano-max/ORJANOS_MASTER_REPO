// This file is the executable in the husky_pick_and_place node.
// It contains an implementation of the husky_pick_and_place class,
// where a scene is read from a .scene file contained in the "params" folder of
// the husky_pick_and_place ros package.
//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "pick_and_place_class.cpp"


int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  static const std::string manipulator_namespace = "vx300";
  static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PickAndPlace>(manipulator_namespace, options);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper;
  move_group_interface_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP_ARM);
  move_group_interface_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP_GRIPPER);
  node->move_group_interface_arm_ = move_group_interface_arm;
  node->move_group_interface_gripper_= move_group_interface_gripper;

  static const rclcpp::Logger LOGGER = node->get_logger();

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 
