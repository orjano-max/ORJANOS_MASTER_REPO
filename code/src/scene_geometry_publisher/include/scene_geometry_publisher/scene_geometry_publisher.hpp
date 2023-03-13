// Header file for the scene_geometry_publisher class
// scene_geometry_publisher.hpp
//
// Author: Ørjan Øvsthus

#ifndef SCENE_GEOMETRY_PUBLISHER_H
#define SCENE_GEOMETRY_PUBLISHER_H

#include <memory>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


class ScenePublisher : public rclcpp::Node
{
  public:

    ScenePublisher(const rclcpp::NodeOptions & options);

    void load_scene();

    std::vector<moveit_msgs::msg::CollisionObject> getCollisionObjects();

    std::string getFrameId();

    std::string getSceneName();

    std::string getSceneFilePath();

    void setFrameId(std::string frameId);

    void setSceneName(std::string sceneName);

    void setSceneFilePath(std::string filePath);
    
    void readScenefile();

  private:

    std::string frameId_ = "world";
    std::string sceneName_ = "noname";
    std::string fileName_ = "scene_geometry.scene";
    std::string filePath_;
    std::fstream file_;
    std::vector<moveit_msgs::msg::CollisionObject> collisionObjects_;

    std::vector<std::string> split(const std::string& str, char delim);

    std::vector<std::string> createObjectStringVector(std::string &line);

    moveit_msgs::msg::CollisionObject createObject(std::vector<std::string> objectVector);
};

#endif