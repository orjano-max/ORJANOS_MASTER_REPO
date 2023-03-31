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
      bool success = (move_group_interface_gripper_->plan(my_plan_gripper_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_gripper_->move();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Planning Failed!");
      }

      // --- Double check the position of the thingy ---
      searchForObjectFrame();

      
      // Place the TCP (Tool Center Point, the tip of the robot) directly above the thingy 
      tf2::Quaternion QObj; 
      QObj.setX(object_pose_.pose.orientation.x);
      QObj.setY(object_pose_.pose.orientation.y);
      QObj.setZ(object_pose_.pose.orientation.z);
      QObj.setW(object_pose_.pose.orientation.w);
      tf2::Matrix3x3 objectMat(QObj);
      tf2Scalar objRoll, objRoll0, objRoll1;
      tf2Scalar objPitch, objPitch0, objPitch1;
      tf2Scalar objYaw, objYaw0, objYaw1;
      objectMat.getRPY(objRoll0, objPitch0, objYaw0, 1);
      objectMat.getRPY(objRoll1, objPitch1, objYaw1, 2);

      objRoll = std::min(std::abs(objRoll0), std::abs(objRoll1));
      objPitch = std::min(std::abs(objPitch0), std::abs(objPitch1));
      objYaw = std::min(std::abs(objYaw0), std::abs(objYaw1));


      RCLCPP_INFO(this->get_logger(), "Roll of object: %f", static_cast<float>(objRoll));
      RCLCPP_INFO(this->get_logger(), "Pitch of object: %f", static_cast<float>(objPitch));
      RCLCPP_INFO(this->get_logger(), "Yaw of object: %f", static_cast<float>(objYaw));

      qRot.setRPY(0, 0, objYaw);
      qRot.normalize();
      
      geometry_msgs::msg::Pose above_pose_object;
      above_pose_object.orientation.x = qRot.getX();
      above_pose_object.orientation.y = qRot.getY();
      above_pose_object.orientation.z = qRot.getZ();
      above_pose_object.orientation.w = qRot.getW();
      above_pose_object.position = object_pose_.pose.position;
      above_pose_object.position.z = object_pose_.pose.position.z + 0.2;
      RCLCPP_INFO(this->get_logger(), "Moving to:\n  x= %f y= %f z= %f", above_pose_object.position.x, above_pose_object.position.y, above_pose_object.position.z);
      RCLCPP_INFO(this->get_logger(), "\n x= %f y= %f z= %f w= %f", above_pose_object.orientation.x, above_pose_object.orientation.y, above_pose_object.orientation.z, above_pose_object.orientation.w);
      planAndExecuteArm();

      // Place the TCP (Tool Center Point, the tip of the robot) at the thingy
      
      geometry_msgs::msg::Pose target_pose_at_object;
      target_pose_at_object.orientation = above_pose_object.orientation;
      target_pose_at_object.position = object_pose_.pose.position;
      target_pose_at_object.position.z = object_pose_.pose.position.z;
      move_group_interface_arm_->setPoseTarget(target_pose_at_object);
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
      tf2::Quaternion qRot;
      qRot.setRPY(0, 0, qYaw);
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
      geometry_msgs::msg::Pose search_pose;
      search_pose.orientation.x = qRot.getX();
      search_pose.orientation.y = qRot.getY();
      search_pose.orientation.z = qRot.getZ();
      search_pose.orientation.w = qRot.getW();
      search_pose.position.x = 0.0;
      search_pose.position.y = 0;
      search_pose.position.z = 0.45;
      move_group_interface_arm_->setPoseTarget(search_pose);
      
      planAndExecuteArm();
    
    }

    void goToHoldingPos()
    {

      // Move to holding position
      tf2::Quaternion qRot;
      qRot.setRPY(0, 0, 0);
      qRot.normalize();

      geometry_msgs::msg::Pose holding_pose;
      holding_pose.orientation.x = qRot.getX();
      holding_pose.orientation.y = qRot.getY();
      holding_pose.orientation.z = qRot.getZ();
      holding_pose.orientation.w = qRot.getW();
      holding_pose.position.x = 0;
      holding_pose.position.y = 0;
      holding_pose.position.z = 0.5;
      move_group_interface_arm_->setPoseTarget(holding_pose);
      
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
      RCLCPP_INFO(this->get_logger(), "Looking for object tag: %s", tag_frame.c_str());

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
            RCLCPP_ERROR(this->get_logger(), "Timeout reached while looking for tag!");
            return false;
        }
      }

      if (frame_available)
      {
        // Extract the pose from the transform
        RCLCPP_INFO(this->get_logger(), "Found tag: %s", tag_frame.c_str());
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
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;

    void topic_callback(const std_msgs::msg::String & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      current_action_ = msg.data;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    
    
    
};

