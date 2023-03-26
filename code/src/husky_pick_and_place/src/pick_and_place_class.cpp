// This file is the executable in the husky_pick_and_place node.

//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
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
    
    


    void pickObject()
    {

      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;    


      //  Place the TCP (Tool Center Point, the tip of the robot) over the thingy
      double denumerator = object_pose_.pose.position.x;
      if (denumerator == 0) // Avoiding division by zero
      {
        denumerator = denumerator+0.0000001;
      }
      double qYaw = atan(object_pose_.pose.position.y/denumerator); // calculate rotation
      double qYawd = qYaw*180/pi;
      tf2::Quaternion qRot;
      qRot.setRPY(0, pi/2, qYaw);
      qRot.normalize();
      

      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.x = qRot.getX();
      target_pose1.orientation.y = qRot.getY();
      target_pose1.orientation.z = qRot.getZ();
      target_pose1.orientation.w = qRot.getW();
      target_pose1.position.x = object_pose_.pose.position.x - 0.1*cos(qYaw);
      target_pose1.position.y = object_pose_.pose.position.y - 0.1*sin(qYaw);
      target_pose1.position.z = object_pose_.pose.position.z + 0.2;
      move_group_interface_arm_->setPoseTarget(target_pose1);
      
      
      RCLCPP_INFO(node_->get_logger(), "Object pose");
      RCLCPP_INFO(node_->get_logger(), "X: %f", object_pose_.pose.position.x);
      RCLCPP_INFO(node_->get_logger(), "Y: %f", object_pose_.pose.position.y);
      RCLCPP_INFO(node_->get_logger(), "Z: %f", object_pose_.pose.position.z);
      RCLCPP_INFO(node_->get_logger(), "Xrot: %f", object_pose_.pose.orientation.x);
      RCLCPP_INFO(node_->get_logger(), "Yrot: %f", object_pose_.pose.orientation.y);
      RCLCPP_INFO(node_->get_logger(), "Zrot: %f", object_pose_.pose.orientation.z);
      RCLCPP_INFO(node_->get_logger(), "W: %f", object_pose_.pose.orientation.w);
      RCLCPP_INFO(node_->get_logger(), "Z-rot between base and object[rad]: %f", qYaw);
      RCLCPP_INFO(node_->get_logger(), "Z-rot between base and object[deg]: %f", qYawd);
      RCLCPP_INFO(node_->get_logger(), "Target pose");
      RCLCPP_INFO(node_->get_logger(), "X: %f", target_pose1.position.x);
      RCLCPP_INFO(node_->get_logger(), "Y: %f", target_pose1.position.y);
      RCLCPP_INFO(node_->get_logger(), "Z: %f", target_pose1.position.z);
      RCLCPP_INFO(node_->get_logger(), "Xrot: %f", target_pose1.orientation.x);
      RCLCPP_INFO(node_->get_logger(), "Yrot: %f", target_pose1.orientation.y);
      RCLCPP_INFO(node_->get_logger(), "Zrot: %f", target_pose1.orientation.z);
      RCLCPP_INFO(node_->get_logger(), "W: %f", target_pose1.orientation.w);
      

      bool success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }

      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues("Released"));

      success = (move_group_interface_gripper_->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_gripper_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed for Grasping!");
      }

      // --- Double check the position of the thingy ---
      searchForObjectFrame();

      // 3. Place the TCP (Tool Center Point, the tip of the robot) down to the thingy 
      geometry_msgs::msg::PoseStamped current_pose = move_group_interface_arm_->getCurrentPose();


      RCLCPP_INFO(node_->get_logger(), "Current pose");
      RCLCPP_INFO(node_->get_logger(), "X: %f", current_pose.pose.position.x);
      RCLCPP_INFO(node_->get_logger(), "Y: %f", current_pose.pose.position.y);
      RCLCPP_INFO(node_->get_logger(), "Z: %f", current_pose.pose.position.z);
      RCLCPP_INFO(node_->get_logger(), "Xrot: %f", current_pose.pose.orientation.x);
      RCLCPP_INFO(node_->get_logger(), "Yrot: %f", current_pose.pose.orientation.y);
      RCLCPP_INFO(node_->get_logger(), "Zrot: %f", current_pose.pose.orientation.z);
      RCLCPP_INFO(node_->get_logger(), "W: %f", current_pose.pose.orientation.w);
      
      geometry_msgs::msg::Pose target_pose2;
      
      target_pose2.orientation = target_pose1.orientation;
      target_pose2.position = object_pose_.pose.position;
      target_pose2.position.z = object_pose_.pose.position.z + 0.05;
      move_group_interface_arm_->setPoseTarget(target_pose2);
      
      success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }


      // 4. Grasp the thingy
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues("Grasping"));

      success = (move_group_interface_gripper_->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_gripper_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed for Grasping!");
      }

      // 5. Lift the thingy
      geometry_msgs::msg::Pose target_pose4;
      
      target_pose4.orientation = target_pose1.orientation;
      target_pose4.position = object_pose_.pose.position;
      target_pose4.position.z = object_pose_.pose.position.z + 0.2;

      move_group_interface_arm_->setPoseTarget(target_pose4);


      //  Move to holding position, it is nice for holding stuff
      this->goToHoldingPos();
    
      
    }

    void placeObject()
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

      // 2. Place the TCP (Tool Center Point, the tip of the robot) over the release pos 
      geometry_msgs::msg::Pose target_pose1;
      
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose = move_group_interface_arm_->getCurrentPose();


      tf2::Quaternion qRot;
      qRot.setRPY(0, pi/2, -pi/2);
      qRot.normalize();
     
      target_pose1.position.x = 0.0;
      target_pose1.position.y = -0.5;
      target_pose1.position.z = 0.1;
      target_pose1.orientation.x = qRot.getX();
      target_pose1.orientation.y = qRot.getY();
      target_pose1.orientation.z = qRot.getZ();
      target_pose1.orientation.w = qRot.getW();
      move_group_interface_arm_->setPoseTarget(target_pose1);

      RCLCPP_INFO(node_->get_logger(), "Target pose");
      RCLCPP_INFO(node_->get_logger(), "X: %f", target_pose1.position.x);
      RCLCPP_INFO(node_->get_logger(), "Y: %f", target_pose1.position.y);
      RCLCPP_INFO(node_->get_logger(), "Z: %f", target_pose1.position.z);
      RCLCPP_INFO(node_->get_logger(), "Xrot: %f", target_pose1.orientation.x);
      RCLCPP_INFO(node_->get_logger(), "Yrot: %f", target_pose1.orientation.y);
      RCLCPP_INFO(node_->get_logger(), "Zrot: %f", target_pose1.orientation.z);
      RCLCPP_INFO(node_->get_logger(), "W: %f", target_pose1.orientation.w);
      
      bool success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }

       // 3. Set the thingy down
      current_pose = move_group_interface_arm_->getCurrentPose();
      
      geometry_msgs::msg::Pose target_pose2;
      target_pose2.orientation = current_pose.pose.orientation;
      target_pose2.position= target_pose1.position;
      target_pose2.position.z = target_pose1.position.z - 0.2;
      move_group_interface_arm_->setPoseTarget(target_pose2);
      
      success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
      
      // 2. Release the thingy
      move_group_interface_gripper_->setJointValueTarget(move_group_interface_gripper_->getNamedTargetValues("Released"));

      success = (move_group_interface_gripper_->plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_gripper_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }

      // 2. Place the TCP (Tool Center Point, the tip of the robot) over the release pos 
      move_group_interface_arm_->setPoseTarget(target_pose1);
      
      success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }

      // 3. Move to sleep position
      move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Sleep"));
      
      success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
    }

    void goToHomePos()
    {
       moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

      // Move to home position
      move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Home"));
      
      bool success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
    
    }

    void goToSearchPos()
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

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
      target_pose1.position.z = 0.5;
      move_group_interface_arm_->setPoseTarget(target_pose1);
      
      bool success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
    
    }

    void goToHoldingPos()
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;


      // Move to holding position
      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.w = 1;
      target_pose1.position.x = 0;
      target_pose1.position.y = 0;
      target_pose1.position.z = 0.5;
      move_group_interface_arm_->setPoseTarget(target_pose1);
      
      bool success = (move_group_interface_arm_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        move_group_interface_arm_->move();
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planning Failed!");
      }
    
    }

    void searchForObjectFrame(const double timeout = 10.0)
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
            return;
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


  private:
    rclcpp::Node::SharedPtr node_;
    std::string PLANNING_GROUP_ARM_;
    std::string PLANNING_GROUP_GRIPPER_;
    geometry_msgs::msg::PoseStamped object_pose_;
    
    
};

