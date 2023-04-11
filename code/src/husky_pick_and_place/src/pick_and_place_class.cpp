// This file is the executable in the husky_pick_and_place node.

//
// Author: Ørjan Øvsthus

#include <memory>
#include <fstream>
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
      action_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "action", 10, std::bind(&PickAndPlace::topic_callback, this, std::placeholders::_1));


      publisher_ = this->create_publisher<std_msgs::msg::String>("action_status", 10);

    }
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper_;
    const float pi = std::atan(1)*4.0;

    void pickObject()
    {

      // Publish action status
      this->publish_string("picking");

      // Set current action back to none
      this->current_action_ = "none";

      if (object_pose_ == empty_pose_)
      {
        RCLCPP_ERROR(this->get_logger(),"No object pose stored, aborting picking procedure!");
        RCLCPP_ERROR(this->get_logger(),"Run the 'searchForObject()' function to populate the object_pose_ variable!");
        // Move to sleep position
        move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Sleep"));
        planAndExecuteArm();
        return;
      }

      tf2::Quaternion QObj; 
      QObj.setX(object_pose_.orientation.x);
      QObj.setY(object_pose_.orientation.y);
      QObj.setZ(object_pose_.orientation.z);
      QObj.setW(object_pose_.orientation.w);
      tf2::Matrix3x3 objectMat(QObj);
      tf2Scalar objRoll;
      tf2Scalar objPitch;
      tf2Scalar objYaw;

      objectMat.getRPY(objRoll, objPitch, objYaw);

      //  Place the TCP (Tool Center Point, the tip of the robot) over the thingy, but a little shifted
      double qYaw = computeYawAngle(object_pose_);
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
      target_pose_inspect.position.x = object_pose_.position.x - shift*cos(qYaw);
      target_pose_inspect.position.y = object_pose_.position.y - shift*sin(qYaw);
      target_pose_inspect.position.z = object_pose_.position.z + heightAbove;
      move_group_interface_arm_->setPoseTarget(target_pose_inspect);
      planAndExecuteArm();

      // Open gripper
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues("Released"));
      planAndExecuteGripper();

      // --- Double check the position of the thingy ---
      searchForObject();
      
      // Place the TCP (Tool Center Point, the tip of the robot) directly above the thingy 
      QObj.setX(object_pose_.orientation.x);
      QObj.setY(object_pose_.orientation.y);
      QObj.setZ(object_pose_.orientation.z);
      QObj.setW(object_pose_.orientation.w);
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
      above_pose_object.position = object_pose_.position;
      above_pose_object.position.z = object_pose_.position.z + 0.05;
      RCLCPP_INFO(this->get_logger(), "Moving to above object");
      RCLCPP_INFO(this->get_logger(), "\n  x= %f y= %f z= %f", above_pose_object.position.x, above_pose_object.position.y, above_pose_object.position.z);
      RCLCPP_INFO(this->get_logger(), "\n x= %f y= %f z= %f w= %f", above_pose_object.orientation.x, above_pose_object.orientation.y, above_pose_object.orientation.z, above_pose_object.orientation.w);
      move_group_interface_arm_->setPoseTarget(above_pose_object);
      planAndExecuteArm();

      // Place the TCP (Tool Center Point, the tip of the robot) at the thingy
      
      geometry_msgs::msg::Pose target_pose_at_object;
      target_pose_at_object.orientation = above_pose_object.orientation;
      target_pose_at_object.position = object_pose_.position;
      move_group_interface_arm_->setPoseTarget(target_pose_at_object);
      RCLCPP_INFO(this->get_logger(), "Moving to object");
      planAndExecuteArm();

      // Grasp the thingy
      std::string graspPose = "Grasping_" + current_object_;
      RCLCPP_INFO(this->get_logger(), "Grasping pose: %s", graspPose.c_str());
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues(graspPose));
      planAndExecuteGripper();

      // Lift the thingy
      geometry_msgs::msg::Pose target_pose_lift_object;
      target_pose_lift_object.orientation = above_pose_object.orientation;
      target_pose_lift_object.position = object_pose_.position;
      target_pose_lift_object.position.z = object_pose_.position.z + 0.2;
      move_group_interface_arm_->setPoseTarget(target_pose_lift_object);
      planAndExecuteArm();


      //  Move to holding position, it is nice for holding stuff
      this->goToHoldingPos();

      // Publish action status
      this->publish_string("picking finished");
    
    }

    void placeObject()
    {

      // Publish action status
      this->publish_string("placing");

      // Set current action back to none
      this->current_action_ = "none";

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

      // Publish action status
      this->publish_string("placing finished");
    }

    void calibrate()
    {

      // Publish action status
      this->publish_string("calibrating");

      // Set current action back to none
      this->current_action_ = "none";

      tf2::Quaternion qCalib;
      std::string calib_frame = "calib_tag";
      float rot = 0.0;
      std::vector <geometry_msgs::msg::Pose> measurements;
      geometry_msgs::msg::Pose calib_pose;

      RCLCPP_INFO(this->get_logger(), "Starting calibration procedure...");

      // Set calib position
      calib_pose.position.x = 0.4;
      calib_pose.position.y = 0;
      calib_pose.position.z = 0.3;
  
      // Get the measurements
      for (int i = 0; i<4; i++)
      {
        RCLCPP_INFO(this->get_logger(), "Value of rot: %f", rot);
        qCalib.setRPY(0, pi/2, rot);
        qCalib.normalize();
        calib_pose.orientation.x = qCalib.getX();
        calib_pose.orientation.y = qCalib.getY();
        calib_pose.orientation.z = qCalib.getZ();
        calib_pose.orientation.w = qCalib.getW();
        move_group_interface_arm_->setPoseTarget(calib_pose);
        planAndExecuteArm();
        
        measurements.push_back(this->searchForTagFrame(10.0, calib_frame));

        rot += pi/2;
      }      

      double error_x = (measurements[0].position.x - measurements[2].position.x)/2;
      double error_y = (measurements[1].position.y - measurements[3].position.y)/2;

      RCLCPP_INFO(this->get_logger(), "Adjust the camera position accordingly:");
      RCLCPP_INFO(this->get_logger(), "X: %f", error_x);
      RCLCPP_INFO(this->get_logger(), "Y: %f", error_y);

      // Publish action status
      this->publish_string("calibrating finished");
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

    void searchForObject()
    {
      // This function searches for an object and stores the information about this
      // Object in the member variable object_pose_

      // Get tag pose
      geometry_msgs::msg::Pose tag_pose = searchForTagFrame();
      tf2::Vector3 tag_pos(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z);
      tf2::Quaternion tag_rot(tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w);


      // Create a transform from the object's frame to the middle of the object
      tf2::Quaternion rotation(0, 0, 0, 1);
      tf2::Vector3 translation(0, 0, -0.05);  // Shift along the z-axis to the middle of the object
      tf2::Transform objectToMiddle(rotation, translation); // Apply rotation to the translation
      
      tf2::Vector3 object_pos = objectToMiddle * tag_pos;  // Apply the transform to the original coordinates of the object's surface

      object_pose_.position.x = object_pos.getX();
      object_pose_.position.y = object_pos.getY();
      object_pose_.position.z = object_pos.getZ();
      object_pose_.orientation = tag_pose.orientation;

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

    std::string getCurrent_object()
    {
      return current_object_;
    }


  private:
  
    bool is_picking_;
    mutable std::string current_action_;
    mutable std::string current_object_ = "case"; // default;
    mutable std::vector<double> object_dimensions_ = {0.0, 0.0, 0.0}; // xyz dimensions of object
    std::string PLANNING_GROUP_ARM_;
    std::string PLANNING_GROUP_GRIPPER_;
    geometry_msgs::msg::Pose object_pose_;
    geometry_msgs::msg::Pose calib_pose_;
    geometry_msgs::msg::Pose empty_pose_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void topic_callback(const std_msgs::msg::String & msg) const
    {

      std::vector<std::string> messages = this->split(msg.data, ' ');

      if (!messages[0].empty())
      {
        RCLCPP_INFO(this->get_logger(), "Got action command: '%s'", messages[0].c_str());
        current_action_ = messages[0];
      }

      if (messages.size() > 1 && !messages[1].empty())
      {
        RCLCPP_INFO(this->get_logger(), "Got object: '%s'", messages[1].c_str());
        current_object_ = messages[1];
      }

      if (messages.size() > 4 && !messages[2].empty() && !messages[3].empty() && !messages[4].empty())
      {
        RCLCPP_INFO(this->get_logger(), "Got dimensions: '%s, %s, %s'", messages[2].c_str(), messages[3].c_str(), messages[4].c_str());
        double object_x = std::stod(messages[2]);
        double object_y = std::stod(messages[3]);
        double object_z = std::stod(messages[4]);
        object_dimensions_ = {object_x, object_y, object_z};
      }

    
    }

    void publish_string(std::string content)
    {
      auto message = std_msgs::msg::String();
      message.data = content;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    std::vector<std::string> split(const std::string& str, char delim) const
    {
      std::vector<std::string> tokens;
      std::stringstream ss(str);
      std::string item;
      while (std::getline(ss, item, delim)) {
          tokens.push_back(item);
      }
      return tokens;
    }
    

    geometry_msgs::msg::Pose searchForTagFrame(const double timeout = 10.0, std::string tag_frame = "")
    {
      /* 
      This function is used to search for an apriltag frame.
      It returns the pose of the frame.
      The two parameters are optional.
      timeout defines how long this function will try to find a frame, default value: 10s
      tag_frame specifies which frame to look for. Default: will use the current_object_ member 
      variable if no frame is defined in function call.
      */ 
      geometry_msgs::msg::Pose pose;
      bool frame_available = false; // Wait for tf2 frame to become available
      geometry_msgs::msg::TransformStamped transform;
      std::string planning_frame = move_group_interface_arm_->getPlanningFrame();
      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr}; // Declare tf2 buffer
      std::unique_ptr<tf2_ros::Buffer> tf_buffer; // Declare transform listener
      
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);


      // Set tag frame if not defined in function call
      if (tag_frame == "")
      {
        tag_frame = current_object_ + "_tag";
      }

      // The pose of object is described as object name plus "_link"
      std::string pose_frame = tag_frame;

      RCLCPP_INFO(this->get_logger(), "Looking for tag: %s", pose_frame.c_str());

      // Get the current time
      rclcpp::Time start_time = rclcpp::Clock().now();

      // Look for frame
      while (rclcpp::ok() && !frame_available) 
      {
        try 
        {
            transform = tf_buffer->lookupTransform(planning_frame, pose_frame, tf2::TimePointZero);
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
            return empty_pose_;
        }
      }
      
      // If nothing was detected, return an empty pose.
      if (!frame_available)
      {
        return empty_pose_;
      }

      pose = setPoseFromTransform(transform);

      // Print information apbout the pose of the frame
      RCLCPP_INFO(this->get_logger(), "Found pose of: %s", pose_frame.c_str());
      RCLCPP_INFO(this->get_logger(), "At pos:");
      RCLCPP_INFO(this->get_logger(), "X: %f", pose.position.x);
      RCLCPP_INFO(this->get_logger(), "Y: %f", pose.position.y);
      RCLCPP_INFO(this->get_logger(), "Z: %f", pose.position.z);
      
      return pose;
    }

    geometry_msgs::msg::Pose setPoseFromTransform(geometry_msgs::msg::TransformStamped transform)
    {
      geometry_msgs::msg::Pose pose;

      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation.x = transform.transform.rotation.x;
      pose.orientation.y = transform.transform.rotation.y;
      pose.orientation.z = transform.transform.rotation.z;
      pose.orientation.w = transform.transform.rotation.w;

      return pose;
    }

    
    
    
};

