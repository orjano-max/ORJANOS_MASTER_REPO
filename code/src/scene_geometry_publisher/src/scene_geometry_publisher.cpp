#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  /* // Declare the parameter for robot description
  node->declare_parameter<std::string>("robot_description");

  // Get the robot description from the parameter server
  std::string robot_description;
  if (!node->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(logger, "Failed to get robot description from parameter server");
    return 1;
  }
  
  // Get the robot model from the parameter server
  std::string robot_model;
  
  if (!node->get_parameter("robot_model", robot_model)){
    RCLCPP_ERROR(logger, "Failed to get robot model from parameter server");
    return 1;
  }

  std::cout << "Robot description is set to: " << robot_description << std::endl
            << "Robot model is set to: " << robot_model << std::endl;
 */


  /* 
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;

  //std::string const group_name = "interbotix_arm";
  std::string const group_name = "panda_arm";

  MoveGroupInterface::Options move_group_options(group_name);
  //auto move_group_interface = MoveGroupInterface(node, move_group_options);
  auto move_group_interface = MoveGroupInterface(node, group_name);
  */


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
  move_group_interface.execute(plan);
  } else {
  RCLCPP_ERROR(logger, "Planing failed!");
  }


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
