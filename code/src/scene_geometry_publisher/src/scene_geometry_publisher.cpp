#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>



class ScenePublisher : public rclcpp::Node
{
  public:

    std::string scene_name_ = "noname";
    std::string file_path_ = "~/git/ORJANOS_MASTER_REPO/code/params/scene_geometry.scene";

    ScenePublisher(const rclcpp::NodeOptions & options) : Node("scene_geometry_publisher", options) {

      std::string joint_state_topic_;

      RCLCPP_INFO(get_logger(), "Scene publisher node started.");

      this->get_parameter("joint_state_topic", joint_state_topic_);

      RCLCPP_INFO(this->get_logger(), "joint_state_topic: %s", joint_state_topic_.c_str());
      
    }

    void setHomePoseTarget(moveit::planning_interface::MoveGroupInterface & move_group_interface)
    {
      // Set a target Pose
      auto const home_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.537;
        msg.position.y = 0.0;
        msg.position.z = 0.427;
        msg.orientation.w = -1;
        return msg;
      }();

      move_group_interface.setPoseTarget(home_pose);
    }

  
  private:
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

};

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
  
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "interbotix_arm";
  
  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  //const moveit::core::JointModelGroup* joint_model_group =
  //    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  node->setHomePoseTarget(move_group_interface);
  

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