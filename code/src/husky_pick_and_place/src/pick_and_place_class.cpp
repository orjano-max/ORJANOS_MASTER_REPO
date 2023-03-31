// This file is the executable in the husky_pick_and_place node.

//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


class PickAndPlace : public rclcpp::Node
{
  public:

    PickAndPlace(std::string move_group_namespace, const rclcpp::NodeOptions & options) 
    : Node("pick_and_place_node", move_group_namespace, options)
    {

      // Create a subscription to listen for the topic "action"
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "action", 10, std::bind(&PickAndPlace::topic_callback, this, std::placeholders::_1));

      // Check if this parameter is set
      if (this->get_parameter("tag_id").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        // Parameter not passed, declare param
        this->declare_parameter("tag_id", "case");
      }

    }
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper_;
    const float pi = std::atan(1)*4.0;

    void pickObject()
    {

      if (object_pose_ == empty_pose_)
      {
        RCLCPP_ERROR(this->get_logger(),"No object pose stored, aborting picking procedure!");
        // Move to sleep position
        move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Sleep"));
        planAndExecuteArm();
        return;
      }

      tf2::Quaternion QObj; 
      QObj.setX(object_pose_.pose.orientation.x);
      QObj.setY(object_pose_.pose.orientation.y);
      QObj.setZ(object_pose_.pose.orientation.z);
      QObj.setW(object_pose_.pose.orientation.w);
      tf2::Matrix3x3 objectMat(QObj);
      tf2Scalar objRoll;
      tf2Scalar objPitch;
      tf2Scalar objYaw;

      objectMat.getRPY(objRoll, objPitch, objYaw);

      //  Place the TCP (Tool Center Point, the tip of the robot) over the thingy, but a little shifted
      double qYaw = computeYawAngle(object_pose_.pose);
      tf2::Quaternion qInspect;
      qInspect.setRPY(0, pi/2, objYaw);
      qInspect.normalize();

      geometry_msgs::msg::Pose target_pose_inspect;
      double heightAbove = 0.3; // Height above when inspecting object
      double shift = 0.00;       // Shift when inspecting object
      target_pose_inspect.orientation.x = qInspect.getX();
      target_pose_inspect.orientation.y = qInspect.getY();
      target_pose_inspect.orientation.z = qInspect.getZ();
      target_pose_inspect.orientation.w = qInspect.getW();
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
      QObj.setX(object_pose_.pose.orientation.x);
      QObj.setY(object_pose_.pose.orientation.y);
      QObj.setZ(object_pose_.pose.orientation.z);
      QObj.setW(object_pose_.pose.orientation.w);
      objectMat.setRotation(QObj);
      objectMat.getRPY(objRoll, objPitch, objYaw);

      tf2::Quaternion qPick;
      qPick.setRPY(0, pi/2, objYaw);
      qPick.normalize();
      
      geometry_msgs::msg::Pose above_pose_object;
      above_pose_object.orientation.x = qPick.getX();
      above_pose_object.orientation.y = qPick.getY();
      above_pose_object.orientation.z = qPick.getZ();
      above_pose_object.orientation.w = qPick.getW();
      above_pose_object.position = object_pose_.pose.position;
      above_pose_object.position.z = object_pose_.pose.position.z + 0.05;
      RCLCPP_INFO(this->get_logger(), "Moving to above object");
      RCLCPP_INFO(this->get_logger(), "\n  x= %f y= %f z= %f", above_pose_object.position.x, above_pose_object.position.y, above_pose_object.position.z);
      RCLCPP_INFO(this->get_logger(), "\n x= %f y= %f z= %f w= %f", above_pose_object.orientation.x, above_pose_object.orientation.y, above_pose_object.orientation.z, above_pose_object.orientation.w);
      move_group_interface_arm_->setPoseTarget(above_pose_object);
      planAndExecuteArm();

      // Place the TCP (Tool Center Point, the tip of the robot) at the thingy
      
      geometry_msgs::msg::Pose target_pose_at_object;
      target_pose_at_object.orientation = above_pose_object.orientation;
      target_pose_at_object.position = object_pose_.pose.position;
      move_group_interface_arm_->setPoseTarget(target_pose_at_object);
      RCLCPP_INFO(this->get_logger(), "Moving to object");
      planAndExecuteArm();

      // Grasp the thingy
      std::string graspPose = "Grasping_" + this->get_parameter("tag_id").as_string();
      RCLCPP_INFO(this->get_logger(), "Grasping pose: %s", graspPose.c_str());
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues(graspPose));
      planAndExecuteGripper();

      // Lift the thingy
      geometry_msgs::msg::Pose target_pose_lift_object;
      target_pose_lift_object.orientation = above_pose_object.orientation;
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
      tf2::Quaternion qPlace;
      qPlace.setRPY(0, pi/2, qYaw);
      qPlace.normalize();
      place_pose.orientation.x = qPlace.getX();
      place_pose.orientation.y = qPlace.getY();
      place_pose.orientation.z = qPlace.getZ();
      place_pose.orientation.w = qPlace.getW();

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

      tf2::Quaternion qSearch;
      qSearch.setRPY(0, pi/4, 0);
      qSearch.normalize();

      // Move to search position
      geometry_msgs::msg::Pose search_pose;
      search_pose.orientation.x = qSearch.getX();
      search_pose.orientation.y = qSearch.getY();
      search_pose.orientation.z = qSearch.getZ();
      search_pose.orientation.w = qSearch.getW();
      search_pose.position.x = 0.0;
      search_pose.position.y = 0;
      search_pose.position.z = 0.45;
      move_group_interface_arm_->setPoseTarget(search_pose);
      
      planAndExecuteArm();
    
    }

    void goToHoldingPos()
    {

      // Move to holding position
      tf2::Quaternion qHolding;
      qHolding.setRPY(0, 0, 0);
      qHolding.normalize();

      geometry_msgs::msg::Pose holding_pose;
      holding_pose.orientation.x = qHolding.getX();
      holding_pose.orientation.y = qHolding.getY();
      holding_pose.orientation.z = qHolding.getZ();
      holding_pose.orientation.w = qHolding.getW();
      holding_pose.position.x = 0;
      holding_pose.position.y = 0;
      holding_pose.position.z = 0.5;
      move_group_interface_arm_->setPoseTarget(holding_pose);
      
      planAndExecuteArm();
    
    }

    void goToSleepPos()
    {

      // Move to sleep position
      move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Sleep"));
      planAndExecuteArm();
    
    }

    bool searchForObjectFrame(const double timeout = 10.0)
    {
      // Create tf2 buffer and transform listener
      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      
      // Wait for tf2 frame to become available
      bool frame_available = false;
      geometry_msgs::msg::TransformStamped transform;

      rclcpp::Time start_time = rclcpp::Clock().now();
      std::string tag_frame = this->get_parameter("tag_id").as_string();
      std::string object_frame = tag_frame + "_link";
      RCLCPP_INFO(this->get_logger(), "Looking for object: %s", object_frame.c_str());

      while (rclcpp::ok() && !frame_available) 
      {
        try 
        {
            std::string planning_frame = move_group_interface_arm_->getPlanningFrame();
            transform = tf_buffer->lookupTransform(planning_frame, object_frame, tf2::TimePointZero);
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
            object_pose_ = empty_pose_;
            RCLCPP_ERROR(this->get_logger(), "Timeout reached while looking for tag!");
            return false;
        }
      }

      if (frame_available)
      {
        // Extract the pose from the transform
        RCLCPP_INFO(this->get_logger(), "Found tag: %s", object_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "At pos:");
        RCLCPP_INFO(this->get_logger(), "X: %f", transform.transform.translation.x);
        RCLCPP_INFO(this->get_logger(), "Y: %f", transform.transform.translation.y);
        RCLCPP_INFO(this->get_logger(), "Z: %f", transform.transform.translation.z);

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
        RCLCPP_ERROR(this->get_logger(), "Planning Failed!");
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
        RCLCPP_ERROR(this->get_logger(), "Planning Failed!");
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

    std::string getCurrent_action()
    {
      return current_action_;
    }


  private:
  
    bool is_picking_;
    mutable std::string current_action_;
    std::string PLANNING_GROUP_ARM_;
    std::string PLANNING_GROUP_GRIPPER_;
    geometry_msgs::msg::PoseStamped object_pose_;
    geometry_msgs::msg::PoseStamped empty_pose_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      current_action_ = msg.data;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    
    
    
};

