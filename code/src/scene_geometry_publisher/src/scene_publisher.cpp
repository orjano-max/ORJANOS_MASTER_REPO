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
      this->setSceneFilePath(sharePath + "/scenes/scene_geometry.scene");
      
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

      line_.clear();

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

      // In case there already is a file open
      file_.close();


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
        collision_object.primitive_poses.push_back(pose);
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
        collision_object.primitive_poses.push_back(pose);
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
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO(get_logger(), "Loaded object: %s", collision_object.id.c_str());

        return collision_object;
      }
      else if (objectVector[9] == "mesh")
      {
        shape_msgs::msg::Mesh mesh;
        geometry_msgs::msg::Point points;
        std::array<uint32_t, 3UL> indices;
        shape_msgs::msg::MeshTriangle triangles;

        int nrOfPoints = std::stoi(objectVector[10]);
        int nrOfTriangles = std::stoi(objectVector[11]);

        int firstPointPos = 12;
        int lastPointPos = firstPointPos + nrOfPoints*3 - 1;
        int firstTrianglePos = lastPointPos+1;
        int lastTrianglePos = firstTrianglePos+nrOfTriangles*3 - 1;

        
        /* 
        RCLCPP_INFO(get_logger(), "First point value: %s", objectVector[firstPointPos].c_str());
        RCLCPP_INFO(get_logger(), "At position: %i", firstPointPos);
        RCLCPP_INFO(get_logger(), "Last point value: %s", objectVector[lastPointPos].c_str());
        RCLCPP_INFO(get_logger(), "At position: %i", lastPointPos);

        RCLCPP_INFO(get_logger(), "First triangle value: %s", objectVector[firstTrianglePos].c_str());
        RCLCPP_INFO(get_logger(), "At position: %i", firstTrianglePos);
        RCLCPP_INFO(get_logger(), "Last triangle value: %s", objectVector[lastTrianglePos].c_str());
        RCLCPP_INFO(get_logger(), "At position: %i", lastTrianglePos); 
        */
       

        // Parsing vertices
        for (int i = firstPointPos; i < lastPointPos; i += 3)
        {
          
          points.set__x(std::stod(objectVector[i]));
          points.set__y(std::stod(objectVector[i+1]));
          points.set__z(std::stod(objectVector[i+2]));
          mesh.vertices.push_back(points);
        }

        /* 
        RCLCPP_INFO(get_logger(), "First value in point vector: %f", mesh.vertices.front().x);
        RCLCPP_INFO(get_logger(), "Last value in point vector: %f", mesh.vertices.back().z);
        */

        // Parsing triangles
        for (int i = firstTrianglePos; i < lastTrianglePos; i += 3)
        {
        
          indices[0] = std::stoul(objectVector[i]);
          indices[1] = std::stoul(objectVector[i+1]);
          indices[2] = std::stoul(objectVector[i+2]);
          
          triangles.vertex_indices = indices;
          mesh.triangles.push_back(triangles);
        }

        /* 
        RCLCPP_INFO(get_logger(), "First value in triangle vector: %i", mesh.triangles.front().vertex_indices[0]);
        RCLCPP_INFO(get_logger(), "Last value in triangle vector: %i", mesh.triangles.back().vertex_indices[2]);
         */
        collision_object.mesh_poses.push_back(pose);
        collision_object.meshes.push_back(mesh);
        collision_object.operation = collision_object.ADD;
        RCLCPP_INFO(get_logger(), "Loaded object: %s", collision_object.id.c_str());

        return collision_object;
        
      }

      RCLCPP_WARN(get_logger(), "Unknown shape type for object '%s'", collision_object.id.c_str());
      return collision_object;
      
    }

};
