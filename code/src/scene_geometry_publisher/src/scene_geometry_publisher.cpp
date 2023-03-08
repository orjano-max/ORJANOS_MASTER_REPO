#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>



class ScenePublisher : public rclcpp::Node
{
  public:

    std::string scene_name_ = "noname";
    std::string file_path_;
    std::string file_name_ = "scene_geometry.scene";
    std::string frame_id_ = "world";
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;

    ScenePublisher(const rclcpp::NodeOptions & options) : Node("scene_geometry_publisher", options) {

      std::string joint_state_topic_;
      std::string share_path = ament_index_cpp::get_package_share_directory("husky_group");

      RCLCPP_INFO(get_logger(), "Scene publisher node started.");

      this->get_parameter("joint_state_topic", joint_state_topic_);

      RCLCPP_INFO(this->get_logger(), "joint_state_topic: %s", joint_state_topic_.c_str());

      file_path_ = share_path + "/params/" + file_name_;

      
    }

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

    void load_scene()
    {
      // Read scene geometry from file

      // Declare variables
      std::string line;
      std::vector<std::string> tokens;
      std::vector<std::string> token;
      std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

      // Read the file
      RCLCPP_INFO(get_logger(), "Loading scene...");
      std::ifstream file(file_path_);
      if (!file.is_open())
      {
        RCLCPP_INFO(get_logger(), "Unable to open file: %s", file_path_.c_str());
        return;
      }

      // The first line is scene name and should not be empty
      if (std::getline(file,line))
      { 
        // First line is the scene name
        scene_name_ = line;
        RCLCPP_INFO(get_logger(), "Got Scene Name: %s", scene_name_.c_str());
      }
      else
      {
          // File is empty
          RCLCPP_INFO(get_logger(), "Scene file is empty!");
      }

      // Go through file
      while (std::getline(file, line))
      {
        // Jump over irrelevant lines
        if (line.empty() || line[0] == '#')
          continue;

        // Remove information about the first object if it has reached the next object
        if (tokens.size() > 1 && tokens.back() == "*")
        {
          tokens.erase(tokens.begin(), tokens.end() - 1);
          tokens.shrink_to_fit();
        }

        // Populate tokens vector
        token = split(line, ' ');
        tokens.insert(tokens.end(), token.begin(), token.end());

        // Skip scene name
        if (tokens[0] == scene_name_)
          continue;

        // File ends with a "."
        if (tokens[0] == ".")
          break;

        // Parse object
        if (tokens.size() >= 2 && tokens[0] == "*")
        {
          moveit_msgs::msg::CollisionObject collision_object;
          collision_object.header.frame_id = frame_id_;
          collision_object.id = tokens[1];

          // Parse object pose
          if (tokens.size() >= 10)
          {
            geometry_msgs::msg::Pose pose;
            pose.position.x = std::stod(tokens[2]);
            pose.position.y = std::stod(tokens[3]);
            pose.position.z = std::stod(tokens[4]);
            //RCLCPP_INFO(get_logger(), "Got position of object...");

            pose.orientation.x = std::stod(tokens[5]);
            pose.orientation.y = std::stod(tokens[6]);
            pose.orientation.z = std::stod(tokens[7]);
            pose.orientation.w = std::stod(tokens[8]);
            //RCLCPP_INFO(get_logger(), "Got orientation of object...");
            collision_object.primitive_poses.push_back(pose);
          }       
          // Parse object shape and dimensions
          if (tokens.size() >= 26 && tokens[10] == "box")
          {
            shape_msgs::msg::SolidPrimitive shape;
            shape.type = shape_msgs::msg::SolidPrimitive::BOX;
            shape.dimensions.resize(3);
            shape.dimensions[0] = std::stod(tokens[11]);
            shape.dimensions[1] = std::stod(tokens[12]);
            shape.dimensions[2] = std::stod(tokens[13]);
            collision_object.primitives.push_back(shape);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);
            RCLCPP_INFO_STREAM(get_logger(), "Loaded object: " << collision_object.id);
            tokens.clear();
            tokens.shrink_to_fit();

          }
          else if (tokens.size() >= 25 && tokens[10] == "cylinder")
          {
            shape_msgs::msg::SolidPrimitive shape;
            shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            shape.dimensions.resize(2);
            shape.dimensions[0] = std::stod(tokens[12]);
            shape.dimensions[1] = std::stod(tokens[11]);
            collision_object.primitives.push_back(shape);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);
            RCLCPP_INFO_STREAM(get_logger(), "Loaded object: " << collision_object.id);
            tokens.clear();
            tokens.shrink_to_fit();
          }
          else if (tokens.size() >= 24 && tokens[10] == "sphere")
          {
            shape_msgs::msg::SolidPrimitive shape;
            shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            shape.dimensions.resize(1);
            shape.dimensions[0] = std::stod(tokens[11]);
            collision_object.primitives.push_back(shape);
            collision_object.operation = collision_object.ADD;
            collision_objects.push_back(collision_object);
            RCLCPP_INFO_STREAM(get_logger(), "Loaded object: " << collision_object.id);
            tokens.clear();
            tokens.shrink_to_fit();
          }
          else if (tokens.size() >= 26 && tokens[10] != "sphere" && tokens[10] != "cylinder" && tokens[10] != "box")
          {
            RCLCPP_WARN(get_logger(), "Unknown shape type for object '%s'", collision_object.id.c_str());
            tokens.clear();
            tokens.shrink_to_fit();
          }
        }

        // End of while loop
      }

      // Finished going through file, add objects
      collision_objects_ = collision_objects;

      // End of function
    }

    std::vector<moveit_msgs::msg::CollisionObject> getCollisionObjects()
    {
      // Return the collision objects stored in class
      return collision_objects_;
    }

  private:

  std::vector<std::string> split(const std::string& str, char delim)
  {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
  }

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
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  node->load_scene();

  RCLCPP_INFO(logger, "Add an object into the world");
  planning_scene_interface.applyCollisionObjects(node->getCollisionObjects());


  // Set a target Pose
      auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.position.x = -0.13;
        msg.position.y = -0.3;
        msg.position.z = 0.342;
        return msg;
      }();

      move_group_interface.setPositionTarget(-0.13, -0.3, 0.342);
  
  
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