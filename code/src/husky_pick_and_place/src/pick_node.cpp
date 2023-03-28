/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "std_msgs/msg/string.hpp"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "pick_and_place_class.cpp"

/* class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
}; */


void topic_callback(const std_msgs::msg::String & msg, rclcpp::Logger &logger, rclcpp::Node::SharedPtr node)
{
  RCLCPP_INFO(logger, "I heard: '%s'", msg.data.c_str());
}


int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  static const std::string manipulator_namespace = "vx300";

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("pick_node", manipulator_namespace, options);

  if (node->get_parameter("tag_id").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    // Parameter not passed, declare param
    node->declare_parameter("tag_id", "case");
  }

  static const rclcpp::Logger LOGGER = node->get_logger();


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

  node->create_subscription<std_msgs::msg::String>(
      "action", 10, std::bind(&topic_callback, node, std::placeholders::_1, LOGGER, node));
  

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  /* rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach(); */

  /* // Initiating the pick and place class
  PickAndPlace pick_and_place_class = PickAndPlace(node);
  
  // Print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", pick_and_place_class.move_group_interface_arm_->getPlanningFrame().c_str());

  // Print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", pick_and_place_class.move_group_interface_arm_->getEndEffectorLink().c_str());

  // Go to position for scanning
  //pick_and_place_class.goToSearchPos();

  // Create tf2 buffer and transform listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Wait for tf2 frame to become available
  bool frame_available = false;
  geometry_msgs::msg::TransformStamped transform;
  // Tag frame is the same as tag_id
  std::string tag_frame = node->get_parameter("tag_id").as_string(); */

  
  RCLCPP_INFO(LOGGER, "Waiting for pick or place command...");

  rclcpp::spin(node);

  // // Join the executor thread before shutting down the node
  //executor.cancel();

  rclcpp::shutdown();
  return 0;
} 



