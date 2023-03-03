
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("hello_moveit", node_options);
  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "interbotix_arm";
  
  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group/include/moveit/move_group/move_group.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  //const moveit::core::JointModelGroup* joint_model_group =
  //    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.6;
    msg.position.y = 0.0;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}


/* void load_scene(std::string scene_file)
    {
      // Read scene geometry from file
      std::ifstream file(file_name_);
      std::string line;
      std::vector<std::string> tokens;
      std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

      ///scene_name = tokens[0];

      while (std::getline(file, line))
      {
        if (line.empty() || line[0] == '#')
          continue;
        boost::split(tokens, line, boost::is_any_of(" "));
        if (tokens[0] == scene_name_;)
          continue;
        else if (tokens[0] == "*")
        {
          moveit_msgs::msg::CollisionObject collision_object;
          collision_object.id = tokens[1];
          geometry_msgs::msg::Pose pose;
          pose.position.x = std::stof(tokens[2]);
          pose.position.y = std::stof(tokens[3]);
          pose.position.z = std::stof(tokens[4]);
          pose.orientation.x = std::stof(tokens[5]);
          pose.orientation.y = std::stof(tokens[6]);
          pose.orientation.z = std::stof(tokens[7]);
          pose.orientation.w = std::stof(tokens[8]);
          shape_msgs::msg::SolidPrimitive shape;
          if (tokens[9] == "box")
            shape.type = shape_msgs::msg::SolidPrimitive::BOX;
          else if (tokens[9] == "cylinder")
            shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          shape.dimensions.resize(3);
          shape.dimensions[0] = std::stof(tokens[10]);
          shape.dimensions[1] = std::stof(tokens[11]);
          if (shape.type == shape_msgs::msg::SolidPrimitive::BOX)
            shape.dimensions[2] = std::stof(tokens[12]);
          else if (shape.type == shape_msgs::msg::SolidPrimitive::CYLINDER)
            shape.dimensions[2] = 0.0;
          collision_object.primitives.push_back(shape);
          collision_object.primitive_poses.push_back(pose);
          collision_object.operation = moveit_msgs::msg::CollisionObjectOperation::ADD;
          collision_objects.push_back(collision_object);
        }
      }

    } */