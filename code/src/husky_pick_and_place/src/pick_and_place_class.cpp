// This file is the executable in the husky_pick_and_place node.

//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


class PickAndPlace
{
  public:

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper_;
    const float pi = std::atan(1)*4.0;

    PickAndPlace(rclcpp::Node::SharedPtr node, std::string PLANNING_GROUP_ARM = "interbotix_arm", std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper")
      : node_(node), PLANNING_GROUP_ARM_(PLANNING_GROUP_ARM), PLANNING_GROUP_GRIPPER_(PLANNING_GROUP_GRIPPER)
      {
        move_group_interface_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_ARM_);
        move_group_interface_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_GRIPPER_);

        
      }
    
    
    void waitForCommand()
    {
      subscription_ = node_->create_subscription<std_msgs::msg::String>(
          "manipulator_command", 10, std::bind(&PickAndPlace::topic_callback, this, std::placeholders::_1));

      while (rclcpp::ok() && message_data_ =="")
      {
        // No message on topic, wait and try again
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        // Subscribe to the "manipulator_command" topic
        subscription_ = node_->create_subscription<std_msgs::msg::String>(
          "manipulator_command", 10, std::bind(&PickAndPlace::topic_callback, this, std::placeholders::_1));
      }

    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg->data.c_str());

      message_data_ = msg->data;
    }

    void pickObject()
    {

      //  Place the TCP (Tool Center Point, the tip of the robot) over the thingy, but a little shifted
      double qYaw = computeYawAngle(object_pose_.pose);
      tf2::Quaternion qRot;
      qRot.setRPY(0, pi/2, qYaw);
      qRot.normalize();

      geometry_msgs::msg::Pose target_pose_inspect;
      double heightAbove = 0.3; // Height above when inspecting object
      double shift = 0.1;       // Shift when inspecting object
      target_pose_inspect.orientation.x = qRot.getX();
      target_pose_inspect.orientation.y = qRot.getY();
      target_pose_inspect.orientation.z = qRot.getZ();
      target_pose_inspect.orientation.w = qRot.getW();
      target_pose_inspect.position.x = object_pose_.pose.position.x - shift*cos(qYaw);
      target_pose_inspect.position.y = object_pose_.pose.position.y - shift*sin(qYaw);
      target_pose_inspect.position.z = object_pose_.pose.position.z + heightAbove;
      move_group_interface_arm_->setPoseTarget(target_pose_inspect);
      planAndExecuteArm();

      // Open gripper
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues("Released"));
      planAndExecuteGripper();

      // --- Double check the position of the thingy ---
      searchForObjectFrame();

      // Place the TCP (Tool Center Point, the tip of the robot) directly above the thingy 
      geometry_msgs::msg::Pose above_pose_object;
      above_pose_object.orientation = target_pose_inspect.orientation;
      above_pose_object.position = object_pose_.pose.position;
      above_pose_object.position.z = object_pose_.pose.position.z + 0.2;
      planAndExecuteArm();

      // Place the TCP (Tool Center Point, the tip of the robot) at the thingy
      
      geometry_msgs::msg::Pose target_pose_at_object;
      target_pose_at_object.orientation = target_pose_inspect.orientation;
      target_pose_at_object.position = object_pose_.pose.position;
      target_pose_at_object.position.z = object_pose_.pose.position.z + 0.05;
      move_group_interface_arm_->setPoseTarget(target_pose_at_object);
      planAndExecuteArm();

      // Grasp the thingy
      std::string graspPose = "Grasping_" + node_->get_parameter("tag_id").as_string();
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues(graspPose));
      planAndExecuteGripper();

      // Lift the thingy
      geometry_msgs::msg::Pose target_pose_lift_object;
      target_pose_lift_object.orientation = target_pose_inspect.orientation;
      target_pose_lift_object.position = object_pose_.pose.position;
      target_pose_lift_object.position.z = object_pose_.pose.position.z + 0.2;
      move_group_interface_arm_->setPoseTarget(target_pose_lift_object);
      planAndExecuteArm();


      //  Move to holding position, it is nice for holding stuff
      this->goToHoldingPos();
    
    }

    void placeObject()
    {
      // Define the place pose
      geometry_msgs::msg::Pose place_pose;
      place_pose.position.x = 0.2;
      place_pose.position.y = -0.4;
      place_pose.position.z = -0.1;

      // Computing yaw angle of end effector at place position 
      double qYaw = computeYawAngle(place_pose);

      // Defining the orientation of the end effector
      tf2::Quaternion qRot;
      qRot.setRPY(0, pi/2, qYaw);
      qRot.normalize();
      place_pose.orientation.x = qRot.getX();
      place_pose.orientation.y = qRot.getY();
      place_pose.orientation.z = qRot.getZ();
      place_pose.orientation.w = qRot.getW();

      // Place the TCP (Tool Center Point, the tip of the robot) over the place pos 
      geometry_msgs::msg::Pose above_pose = place_pose;
      above_pose.position.z += 0.2;
      move_group_interface_arm_->setPoseTarget(above_pose);
      planAndExecuteArm();

       // Set the thingy down
      move_group_interface_arm_->setPoseTarget(place_pose);
      planAndExecuteArm();
      
      // Release the thingy
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues("Released"));
      planAndExecuteGripper();

      // Go up to above release position
      move_group_interface_arm_->setPoseTarget(above_pose);
      planAndExecuteArm();

      // Move to sleep position
      move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Sleep"));
      planAndExecuteArm();
    }

    void goToHomePos()
    {
      // Move to home position
      move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Home"));
      planAndExecuteArm();
    }

    void goToSearchPos()
    {

      tf2::Quaternion qRot;
      qRot.setRPY(0, pi/4, 0);
      qRot.normalize();

      // Move to search position
      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.x = qRot.getX();
      target_pose1.orientation.y = qRot.getY();
      target_pose1.orientation.z = qRot.getZ();
      target_pose1.orientation.w = qRot.getW();
      target_pose1.position.x = 0;
      target_pose1.position.y = 0;
      target_pose1.position.z = 0.45;
      move_group_interface_arm_->setPoseTarget(target_pose1);
      
      planAndExecuteArm();
    
    }

    void goToHoldingPos()
    {

      // Move to holding position
      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.w = 1;
      target_pose1.position.x = 0;
      target_pose1.position.y = 0;
      target_pose1.position.z = 0.5;
      move_group_interface_arm_->setPoseTarget(target_pose1);
      
      planAndExecuteArm();
    
    }

    bool searchForObjectFrame(const double timeout = 10.0)
    {
      // Create tf2 buffer and transform listener
      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      tf_buffer = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      
      // Wait for tf2 frame to become available
      bool frame_available = false;
      geometry_msgs::msg::TransformStamped transform;

      rclcpp::Time start_time = rclcpp::Clock().now();
      std::string tag_frame = node_->get_parameter("tag_id").as_string();

      while (rclcpp::ok() && !frame_available) 
      {
        try 
        {
            std::string planning_frame = move_group_interface_arm_->getPlanningFrame();
            transform = tf_buffer->lookupTransform(planning_frame, tag_frame, tf2::TimePointZero);
            frame_available = true;
        } 
        catch (tf2::TransformException& ex) 
        {
            // Frame not available yet, wait and try again
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        // Check if the timeout has been reached
        double elapsed_time = (rclcpp::Clock().now() - start_time).seconds();
        if (elapsed_time >= timeout) {
            // Timeout reached, return false
            RCLCPP_ERROR(node_->get_logger(), "Timeaout reached while looking for tag!");
            return false;
        }
      }

      if (frame_available)
      {
        // Extract the pose from the transform
        RCLCPP_INFO(node_->get_logger(), "Found tag: %s", tag_frame.c_str());
        RCLCPP_INFO(node_->get_logger(), "At pos:");
        RCLCPP_INFO(node_->get_logger(), "X: %f", transform.transform.translation.x);
        RCLCPP_INFO(node_->get_logger(), "Y: %f", transform.transform.translation.y);
        RCLCPP_INFO(node_->get_logger(), "Z: %f", transform.transform.translation.z);

        setObjectPoseFromTransform(transform);
      }

      return frame_available;
    }

    void setObjectPoseFromTransform(geometry_msgs::msg::TransformStamped transform)
    {
      object_pose_.pose.position.x = transform.transform.translation.x;
      object_pose_.pose.position.y = transform.transform.translation.y;
      object_pose_.pose.position.z = transform.transform.translation.z;
      object_pose_.pose.orientation.x = transform.transform.rotation.x;
      object_pose_.pose.orientation.y = transform.transform.rotation.y;
      object_pose_.pose.orientation.z = transform.transform.rotation.z;
      object_pose_.pose.orientation.w = transform.transform.rotation.w;
    }

    void planAndExecuteArm()
    {
      
      bool success = (move_group_interface_arm_->plan(my_plan_arm_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
    }

    void planAndExecuteGripper()
    {
      
      bool success = (move_group_interface_gripper_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success)
      {
        move_group_interface_gripper_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
    }

    double computeYawAngle(geometry_msgs::msg::Pose pose)
    {
      // Computing yaw angle of pose
      // useful for computing end effector orientation

      double denumerator = pose.position.x;
      if (denumerator == 0) // Avoiding division by zero
      {
        denumerator = denumerator+0.000001;
      }
      double yaw = atan(pose.position.y/denumerator); 

      return yaw;
    }

  private:
    bool is_picking_;
    std::string message_data_;
    rclcpp::Node::SharedPtr node_;
    std::string PLANNING_GROUP_ARM_;
    std::string PLANNING_GROUP_GRIPPER_;
    geometry_msgs::msg::PoseStamped object_pose_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    
};

