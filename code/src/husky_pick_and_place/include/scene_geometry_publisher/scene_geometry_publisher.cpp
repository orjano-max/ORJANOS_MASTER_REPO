// Source code for the scene geometry publisher class.
//
// Author: Ørjan Øvsthus

#include "scene_geometry_publisher/scene_geometry_publisher.hpp"


class ScenePublisher : public rclcpp::Node
{
  public:

    ScenePublisher(const rclcpp::NodeOptions & options, std::string sceneFilePath = "", std::string frameId = "") : Node("scene_geometry_publisher", options) {

      std::string joint_state_topic;

      RCLCPP_INFO(this->get_logger(), "Scene publisher node started.");

      this->get_parameter("joint_state_topic", joint_state_topic);
      RCLCPP_INFO(this->get_logger(), "joint_state_topic: %s", joint_state_topic.c_str());

      this->setScenefilePath(sceneFilePath);

      this->readScenefile();
      
    }

    void load_scene()
    {
      // This function loads a scene from a ".scene" file and stores 
      // the collision object information in the class
      // Declare variables
      std::string line;
      std::vector<moveit_msgs::msg::CollisionObject> collisionObjects;
      std::vector<std::string> objectStringVector;
      
      // First line is the scene name
      std::getline(file_,line);
      sceneName_ = line;
      RCLCPP_INFO(get_logger(), "Got Scene Name: %s", sceneName_.c_str());

      while (line[0] == '*')
      {
        objectStringVector = createObjectStringVector(line);
        collisionObjects.push_back(createObject(objectStringVector));
      }

      // Finished going through file_, add objects
      collisionObjects_ = collisionObjects;

      // End of function
    }

    std::vector<moveit_msgs::msg::CollisionObject> getCollisionObjects()
    {
      return collisionObjects_;
    }

    std::string getFrameId()
    {
      return frameId_;
    }

    std::string getSceneName()
    {
      return sceneName_;
    }

    std::string getfilePath()
    {
      return filePath_;
    }

    void setFrameId(std::string frameId)
    {
      frameId_ = frameId;
    }

    void setSceneName(std::string sceneName)
    {
      sceneName_ = sceneName;
    }

    void setScenefilePath(std::string filePath = "")
    {
      // Use default file path if none was given
      if (filePath == "")
      {
        std::string sharePath = ament_index_cpp::get_package_share_directory("scene_geometry_publisher");
        filePath = sharePath + "/params/scene_geometry.scene";
      }

      filePath_ = filePath;
    }
    
    void readScenefile()
    {
      // This function opens a file and stores it in the "file_" member variable

      // Open the file_ in reading mode
      RCLCPP_INFO(get_logger(), "Opening File: %s", filePath_.c_str());

      file_.open(filePath_, std::ios::in);

      if (!file_.is_open())
      {
        RCLCPP_WARN(get_logger(), "Unable to open file_: %s", filePath_.c_str());
      }
    }

  private:

    std::string frameId_ = "world";
    std::string sceneName_ = "noname";
    std::string filePath_;
    std::fstream file_;
    std::vector<moveit_msgs::msg::CollisionObject> collisionObjects_;

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

    std::vector<std::string> createObjectStringVector(std::string &line)
    {
      std::vector<std::string> objectVector;
      std::vector<std::string> token;
    
      // Object ID is next to *
      objectVector[0] = line.erase(0,2);

      // Next line
      std::getline(file_,line);

      // Structure object information in a string vector
      while (line[0] != '*')
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
      collision_object.header.frame_id = frameId_;

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

};
