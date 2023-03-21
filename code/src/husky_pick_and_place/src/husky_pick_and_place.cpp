// This file is the executable in the husky_pick_and_place node.
// It contains an implementation of the husky_pick_and_place class,
// where a scene is read from a .scene file contained in the "params" folder of
// the husky_pick_and_place ros package.
//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


void setHomePoseTarget(moveit::planning_interface::MoveGroupInterface & move_group_interface)
{
  // Set a target Pose
  auto const home_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.537;
    msg.position.y = 0.0;
    msg.position.z = 0.427;
    msg.orientation.w = 1;
    return msg;
  }();

  move_group_interface.setPoseTarget(home_pose);
}


int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({{"joint_state_topic", "vx300/joint_states"}});
  auto node = std::make_shared<rclcpp::Node>("husky_pick_and_place",options);
  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("husky_pick_and_place");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "interbotix_arm";
  
  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);


  move_group_interface.setPositionTarget(-0.13, -0.3, 0.342);


  //setHomePoseTarget(move_group_interface);
  
  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);

  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 