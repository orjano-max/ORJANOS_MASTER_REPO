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



int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  static const std::string manipulator_namespace = "vx300";
  static const std::string PLANNING_GROUP_ARM = "interbotix_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper";

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PickAndPlace>(manipulator_namespace, options);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper;
  move_group_interface_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP_ARM);
  move_group_interface_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP_GRIPPER);
  node->move_group_interface_arm_ = move_group_interface_arm;
  node->move_group_interface_gripper_= move_group_interface_gripper;



  static const rclcpp::Logger LOGGER = node->get_logger();


  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  
  // Print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", node->move_group_interface_arm_->getPlanningFrame().c_str());

  // Print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", node->move_group_interface_arm_->getEndEffectorLink().c_str()); 


  
  RCLCPP_INFO(LOGGER, "Waiting for pick or place command...");

  /* rclcpp::spin(node); */

  while(rclcpp::ok())
  {
    if (node->getCurrent_action() == "pick")
    {
      // Go to position for scanning
      node->goToSearchPos();
      node->searchForObject();
      node->pickObject();
    }
    else if (node->getCurrent_action() == "place")
    {
      node->placeObject();
    }
    else if (node->getCurrent_action() == "calibrate")
    {
      /* 
      This will run a procedure where there will be taken
      four measurements with the wrist of the manipulator rotated
      90 degrees between each measurement. 
      Make sure there is an apriltag named "calib" in the field of view of the camera.
      The difference in the four measurements will be used to calculate how much the "camera_link" 
      should be moved in order to get correct measurements.
      the "camera_link could be moved physically or in the 
      urdf file/transform where its location is defined.
      "*/
      node->calibrate();
    }
    else if (node->getCurrent_action() == "sleep")
    {
      node->goToSleepPos();
    }

  }

  // // Join the executor thread before shutting down the node
  executor.cancel();

  rclcpp::shutdown();
  return 0;
} 



