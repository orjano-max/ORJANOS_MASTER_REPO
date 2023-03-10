#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>



class ScenePublisher : public rclcpp::Node
{
  public:

    std::string frame_id_ = "world";
    std::string sceneName_ = "noname";
    std::string fileName_ = "scene_geometry.scene";
    std::string filePath_;
    std::fstream file_;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;

    ScenePublisher(const rclcpp::NodeOptions & options) : Node("scene_geometry_publisher", options) {

      std::string joint_state_topic;
      std::string sharePath = ament_index_cpp::get_package_share_directory("husky_group");

      RCLCPP_INFO(this->get_logger(), "Scene publisher node started.");

      this->get_parameter("joint_state_topic", joint_state_topic);
      RCLCPP_INFO(this->get_logger(), "joint_state_topic: %s", joint_state_topic.c_str());

      filePath_ = sharePath + "/params/" + fileName_;

      readScenefile();
      
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

      // Declare variables
      std::string line;
      std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
      std::vector<std::string> objectvector;
      
      // First line is the scene name
      std::getline(file_,line);
      sceneName_ = line;
      RCLCPP_INFO(get_logger(), "Got Scene Name: %s", sceneName_.c_str());

      while (line[0] == '*')
      {
        objectvector = createObjectVector(line);
        collision_objects.push_back(createObject(objectvector));
      }

      // Finished going through file_, add objects
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

    void readScenefile(std::string sceneFile = "")
    {
      // This function reads the scene file_

      std::string filePath = filePath_;

      // Set custom file path if given
      if (sceneFile != "")
      {
        filePath = sceneFile;
      }

      // Read the file_
      RCLCPP_INFO(get_logger(), "Opening File: %s", filePath.c_str());

      file_.open(filePath, std::ios::in);

      if (!file_.is_open())
      {
        RCLCPP_WARN(get_logger(), "Unable to open file_: %s", filePath.c_str());
      }
    }

    std::vector<std::string> createObjectVector(std::string &line)
    {
      std::vector<std::string> objectVector;
      std::vector<std::string> token;
    
      // Object ID is next to *
      objectVector[0] = line.erase(0,2);

      // Next line
      std::getline(file_,line);

      // Structure object information in a string vector
      while (isNumber(token[0]))
      {
      // Populate tokens vector
      token = split(line, ' ');
      token.insert(objectVector.end(), token.begin(), token.end());
      std::getline(file_,line);
      }
      
      return objectVector;
    }

    moveit_msgs::msg::CollisionObject createObject(std::vector<std::string> objectVector)
    {
      moveit_msgs::msg::CollisionObject collision_object;

      collision_object.id = objectVector[0];
      collision_object.header.frame_id = frame_id_;

      // Parse object pose
      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stod(objectVector[0]);
      pose.position.y = std::stod(objectVector[1]);
      pose.position.z = std::stod(objectVector[2]);
      //RCLCPP_INFO(get_logger(), "Got position of object...");
      pose.orientation.x = std::stod(objectVector[3]);
      pose.orientation.y = std::stod(objectVector[4]);
      pose.orientation.z = std::stod(objectVector[5]);
      pose.orientation.w = std::stod(objectVector[6]);
      //RCLCPP_INFO(get_logger(), "Got orientation of object...");
      collision_object.primitive_poses.push_back(pose);

      // Parse object shape and dimensions
      shape_msgs::msg::SolidPrimitive shape;
      if (objectVector[8] == "box")
      {
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = std::stod(objectVector[9]);
        shape.dimensions[1] = std::stod(objectVector[10]);
        shape.dimensions[2] = std::stod(objectVector[11]);
        collision_object.primitives.push_back(shape);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO_STREAM(get_logger(), "Loaded object: " << collision_object.id);

        return collision_object;
      }
      else if (objectVector[8] == "cylinder")
      {
        shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        shape.dimensions.resize(2);
        shape.dimensions[0] = std::stod(objectVector[10]);
        shape.dimensions[1] = std::stod(objectVector[9]);
        collision_object.primitives.push_back(shape);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO_STREAM(get_logger(), "Loaded object: " << collision_object.id);

        return collision_object;
      }
      else if (objectVector[8] == "sphere")
      {
        shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        shape.dimensions.resize(1);
        shape.dimensions[0] = std::stod(objectVector[9]);
        collision_object.primitives.push_back(shape);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO_STREAM(get_logger(), "Loaded object: " << collision_object.id);

        return collision_object;
      }
      RCLCPP_WARN(get_logger(), "Unknown shape type for object '%s'", collision_object.id.c_str());
      
    }

    bool isNumber(const std::string& str) {
      bool is_number = true;
      for (char c : str) {
          if (!isdigit(c)) { // Check if the character is not a digit
              is_number = false;
              break;
          }
      }
      return is_number;
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