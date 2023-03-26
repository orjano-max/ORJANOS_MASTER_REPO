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

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("place_node", manipulator_namespace, options);
  node->declare_parameter("tag_id", "tag_0");
  static const rclcpp::Logger LOGGER = node->get_logger();

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initiating the pick and place class
  PickAndPlace pick_and_place_class = PickAndPlace(node);
  

  pick_and_place_class.placeObject();
  


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 
