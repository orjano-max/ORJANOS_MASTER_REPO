// This file is the executable in the husky_pick_and_place node.
// It contains an implementation of the husky_pick_and_place class,
// where a scene is read from a .scene file contained in the "params" folder of
// the husky_pick_and_place ros package.
//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>




static const rclcpp::Logger LOGGER = rclcpp::get_logger("husky_pick_and_place");

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("husky_pick_and_place","vx300",options);

  
  // Create a ROS logger

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Joint state topic in node: %s", node->get_parameter("joint_state_topic").as_string().c_str());
  
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface_arm/include/moveit/move_group_interface_arm/move_group_interface_arm.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(node, PLANNING_GROUP_GRIPPER);


  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_interface_arm.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_interface_arm.getJointModelGroupNames().begin(), move_group_interface_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
 
  // 1. Move to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("Home"));
  
  bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_arm.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }



  // 2. Place the TCP (Tool Center Point, the tip of the robot) to the side of the thingy 
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose = move_group_interface_arm.getCurrentPose();
  
  geometry_msgs::msg::Pose target_pose1;
  
  target_pose1.orientation = current_pose.pose.orientation;
  target_pose1.position.x = current_pose.pose.position.x + 0.2;
  target_pose1.position.y = current_pose.pose.position.y;
  target_pose1.position.z = current_pose.pose.position.z - 0.2;
  move_group_interface_arm.setPoseTarget(target_pose1);
  
  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_arm.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }

  // 3. Place the TCP (Tool Center Point, the tip of the robot) on the thingy 
  current_pose = move_group_interface_arm.getCurrentPose();
  
  geometry_msgs::msg::Pose target_pose2;
  
  target_pose2.orientation = current_pose.pose.orientation;
  target_pose2.position.x = current_pose.pose.position.x + 0.2;
  target_pose2.position.y = current_pose.pose.position.y;
  target_pose2.position.z = current_pose.pose.position.z;
  move_group_interface_arm.setPoseTarget(target_pose2);
  
  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_arm.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }

  // 4. Grasp the thingy
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("Grasping"));

  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_gripper.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }


  // 5. Move to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("Home"));
  
  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_arm.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }

  // 6. Release the thingy
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("Released"));

  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_gripper.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }


  // 7. Move to sleep position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("Sleep"));
  
  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    move_group_interface_arm.move();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning Failed!");
  }
  



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 