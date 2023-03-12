// Source code for the scene geometry publisher class.
//
// Author: Ørjan Øvsthus

#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


class ScenePublisher : public rclcpp::Node
{
  public:

    ScenePublisher(const rclcpp::NodeOptions & options) : Node("scene_geometry_publisher", options)
    {
      // Declare sharepath to default scene geometry publisher .scene file
      std::string sharePath = ament_index_cpp::get_package_share_directory("scene_geometry_publisher");

      RCLCPP_INFO(this->get_logger(), "Scene publisher node initiated.");

      
      // Set default file path
      this->setSceneFilePath(sharePath + "/params/scene_geometry.scene");
      
    }

    void load_scene()
    {
      // This function loads a scene from a ".scene" file and stores 
      // the collision object information in the class
      // Declare variables
      std::vector<moveit_msgs::msg::CollisionObject> collisionObjects;
      std::vector<std::string> objectStringVector;
      std::vector<std::string> token;
      
      // First line_ is the scene name
      std::getline(file_,line_);
      sceneName_ = line_;
      RCLCPP_INFO(get_logger(), "Got Scene Name: %s", sceneName_.c_str());

      // Get next line_
      std::getline(file_,line_);
      token = split(line_, ' ');
      while (token[0] == "*")
      {
        objectStringVector = createObjectStringVector();
        collisionObjects.push_back(createObject(objectStringVector));
        token = split(line_, ' ');
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

    std::string getSceneFilePath()
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

    void setSceneFilePath(std::string filePath)
    {
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
    std::string line_;
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

    std::vector<std::string> createObjectStringVector()
    {
      std::vector<std::string> objectVector;
      std::vector<std::string> token;
    
      // Object ID is next to *
      objectVector.push_back(line_.erase(0,2));

      // Next line_
      std::getline(file_,line_);
      token = split(line_, ' ');
      // Structure object information in a string vector
      while (token[0] != "*" && token[0] != ".")
      {
      // Populate object vector
      objectVector.insert(objectVector.end(), token.begin(), token.end());
      std::getline(file_,line_);
      token = split(line_, ' ');
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
      pose.position.x = std::stod(objectVector[1]);
      pose.position.y = std::stod(objectVector[2]);
      pose.position.z = std::stod(objectVector[3]);
      //RCLCPP_INFO(get_logger(), "Got position of object...");
      pose.orientation.x = std::stod(objectVector[4]);
      pose.orientation.y = std::stod(objectVector[5]);
      pose.orientation.z = std::stod(objectVector[6]);
      pose.orientation.w = std::stod(objectVector[7]);
      //RCLCPP_INFO(get_logger(), "Got orientation of object...");
      collision_object.primitive_poses.push_back(pose);

      // Parse object shape and dimensions
      shape_msgs::msg::SolidPrimitive shape;
      if (objectVector[9] == "box")
      {
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = std::stod(objectVector[10]);
        shape.dimensions[1] = std::stod(objectVector[11]);
        shape.dimensions[2] = std::stod(objectVector[12]);
        collision_object.primitives.push_back(shape);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO(get_logger(), "Loaded object: %s", collision_object.id.c_str());

        return collision_object;
      }
      else if (objectVector[9] == "cylinder")
      {
        shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        shape.dimensions.resize(2);
        shape.dimensions[0] = std::stod(objectVector[11]);
        shape.dimensions[1] = std::stod(objectVector[10]);
        collision_object.primitives.push_back(shape);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO(get_logger(), "Loaded object: %s", collision_object.id.c_str());

        return collision_object;
      }
      else if (objectVector[9] == "sphere")
      {
        shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        shape.dimensions.resize(1);
        shape.dimensions[0] = std::stod(objectVector[10]);
        collision_object.primitives.push_back(shape);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO(get_logger(), "Loaded object: %s", collision_object.id.c_str());

        return collision_object;
      }

      RCLCPP_WARN(get_logger(), "Unknown shape type for object '%s'", collision_object.id.c_str());
      return collision_object;
      
    }

};
