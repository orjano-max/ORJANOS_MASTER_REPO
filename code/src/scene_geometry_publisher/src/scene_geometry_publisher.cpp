// This file is the executable in the scene_geometry_publisher node.
// It contains an implementation of the scene_geometry_publisher class
// where a scene is read from a .scene file contained in the "params" folder of
// the scene_geometry_publisher ros package and collision objects are published to the planning scene.
// It also illustrates how collision objects can be read from a .scene file and published as an
// AttatchedCollision object to the end effector of a robot.
//
// Author: Ørjan Øvsthus

#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "scene_publisher.cpp"


 int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({{"joint_state_topic", "vx300/joint_states"}});
  auto node = std::make_shared<ScenePublisher>(options);
  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("scene_geometry_publisher");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  

  // Read the .scene file at default scene file path. Other paths can be set using "node->setSceneFilePath()"
  node->readScenefile();

  // Load the information from the .scene file into the node
  node->load_scene();

  // Deploy loaded planning scene
  RCLCPP_INFO(logger, "Adding objects into the world");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(node->getCollisionObjects());

  // ------------
  // Now we will load the realsense camera assembly and attatch it to the gripper
  // ----------- 
  // Set frame id to gripper link
  /* std::string frameId = "vx300/gripper_link";
  node->setFrameId(frameId);
 
  // Set path to the realsense.scene file
  std::string packagePath = ament_index_cpp::get_package_share_directory("scene_geometry_publisher");
  node->setSceneFilePath(packagePath +  "/scenes/realsense.scene");

  // Read the realsense scene file
  node->readScenefile();

  // Load the information from the .scene file into the node
  node->load_scene();

  // Deploy loaded objects to the robot
  RCLCPP_INFO(logger, "Applying objects onto the robot");
  std::vector<moveit_msgs::msg::CollisionObject> collisionObjects = node->getCollisionObjects();
  std::vector<moveit_msgs::msg::AttachedCollisionObject> attachedObjects;

  for (int i = 0; i < static_cast<int>(collisionObjects.size()); i++)
  {
    moveit_msgs::msg::AttachedCollisionObject attachedObject;
    attachedObject.object = collisionObjects[i];
    attachedObject.link_name = node->getFrameId();
    attachedObject.object.operation = attachedObject.object.ADD;
    attachedObject.touch_links = std::vector<std::string>{attachedObject.link_name};
    attachedObjects.push_back(attachedObject);
  }

  planning_scene_interface.applyAttachedCollisionObjects(attachedObjects); */



  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 