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

    PickAndPlace(rclcpp::Node::SharedPtr node, std::string PLANNING_GROUP_ARM = "interbotix_arm", std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper")
      : node_(node), PLANNING_GROUP_ARM_(PLANNING_GROUP_ARM), PLANNING_GROUP_GRIPPER_(PLANNING_GROUP_GRIPPER)
      {
        
        

        move_group_interface_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_ARM_);
        move_group_interface_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_GRIPPER_);


      }
    
  

    void pickObject(geometry_msgs::msg::Transform &transform)
    {
    
      geometry_msgs::msg::PoseStamped object_pose;
      object_pose.pose.position.x = transform.translation.x;
      object_pose.pose.position.y = transform.translation.y;
      object_pose.pose.position.z = transform.translation.z;
      object_pose.pose.orientation = transform.rotation;

      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    
      // 1. Starting at search position
      this->goToSearchPos();

      geometry_msgs::msg::PoseStamped current_pose;

      // 2. Place the TCP (Tool Center Point, the tip of the robot) over the thingy 
      /* 
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose = move_group_interface_arm_->getCurrentPose();

      tf2::Quaternion qCurrent;  
      qCurrent.setX(current_pose.pose.orientation.x);
      qCurrent.setY(current_pose.pose.orientation.y);
      qCurrent.setZ(current_pose.pose.orientation.z);
      qCurrent.setW(current_pose.pose.orientation.w);

      tf2::Quaternion qRot;
      qRot.setRPY(3.141592/2, 0, 0);
      
      tf2::Quaternion qNew = qRot*qCurrent;
      qNew.normalize(); 
      */
      
      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.w = 1;
      target_pose1.position.x = object_pose.pose.position.x - 0.05;
      target_pose1.position.y = object_pose.pose.position.y;
      target_pose1.position.z = object_pose.pose.position.z + 0.2;
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

      // 3. Place the TCP (Tool Center Point, the tip of the robot) down to the thingy 
      current_pose = move_group_interface_arm_->getCurrentPose();
      
      geometry_msgs::msg::Pose target_pose2;
      
      target_pose2.orientation = current_pose.pose.orientation;
      target_pose2.position = current_pose.pose.position;
      target_pose2.position.z = current_pose.pose.position.z - 0.2;
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

      // 3. Place the TCP (Tool Center Point, the tip of the robot) on the thingy 
      current_pose = move_group_interface_arm_->getCurrentPose();
      
      geometry_msgs::msg::Pose target_pose3;
      
      target_pose3.orientation = current_pose.pose.orientation;
      target_pose3.position = current_pose.pose.position;
      target_pose3.position.x = current_pose.pose.position.x + 0.05;
      move_group_interface_arm_->setPoseTarget(target_pose3);
      
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
      
      target_pose4.orientation = current_pose.pose.orientation;
      target_pose4.position = object_pose.pose.position;
      target_pose4.position.z = object_pose.pose.position.z + 0.2;

      move_group_interface_arm_->setPoseTarget(target_pose4);


      // 5. Move to home position
      move_group_interface_arm_->setJointValueTarget(move_group_interface_arm_->getNamedTargetValues("Home"));
      
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

    void placeObject()
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

      // 2. Place the TCP (Tool Center Point, the tip of the robot) on over the release pos 
      geometry_msgs::msg::Point target_pos1;
      target_pos1.x = 0.0;
      target_pos1.y = -0.5;
      target_pos1.z = 0.4;
      move_group_interface_arm_->setPositionTarget(target_pos1.x, target_pos1.y, target_pos1.z);
      
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
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose = move_group_interface_arm_->getCurrentPose();
      
      geometry_msgs::msg::Pose target_pose2;
      target_pose2.orientation = current_pose.pose.orientation;
      target_pose2.position.x = current_pose.pose.position.x;
      target_pose2.position.y = current_pose.pose.position.y;
      target_pose2.position.z = current_pose.pose.position.z - 0.2;
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
      move_group_interface_arm_->setPositionTarget(target_pos1.x, target_pos1.y, target_pos1.z);
      
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

      // Move to search position
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


  private:
    rclcpp::Node::SharedPtr node_;
    std::string PLANNING_GROUP_ARM_;
    std::string PLANNING_GROUP_GRIPPER_;
    
    
};




static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  static const std::string manipulator_namespace = "vx300";

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("pick_and_place_node", manipulator_namespace, options);
  node->declare_parameter("tag_id", "tag_0");
  static const rclcpp::Logger LOGGER = node->get_logger();

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Initiating the pick and place class
  PickAndPlace pick_and_place_class = PickAndPlace(node);
  
  // Print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", pick_and_place_class.move_group_interface_arm_->getPlanningFrame().c_str());

  // Print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", pick_and_place_class.move_group_interface_arm_->getEndEffectorLink().c_str());

  // Go to position for scanning
  pick_and_place_class.goToSearchPos();

  // Create tf2 buffer and transform listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Wait for tf2 frame to become available
  bool frame_available = false;
  geometry_msgs::msg::TransformStamped transform;
  // Tag frame is the same as tag_id
  std::string tag_frame = node->get_parameter("tag_id").as_string();
  RCLCPP_INFO(LOGGER, "Looking for: %s", tag_frame.c_str());
  while (rclcpp::ok() && !frame_available) 
  {
      try 
      {
          std::string planning_frame = pick_and_place_class.move_group_interface_arm_->getPlanningFrame();
          transform = tf_buffer->lookupTransform(tag_frame, planning_frame, tf2::TimePointZero);
          frame_available = true;
      } catch (tf2::TransformException& ex) 
      {
          // Frame not available yet, wait and try again
          rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
  }

  if (frame_available)
  {
    // Extract the pose from the transform
    RCLCPP_INFO(LOGGER, "Found tag: %s", tag_frame.c_str());
    RCLCPP_INFO(LOGGER, "At pos:");
    RCLCPP_INFO(LOGGER, "X: %f", transform.transform.translation.x);
    RCLCPP_INFO(LOGGER, "Y: %f", transform.transform.translation.y);
    RCLCPP_INFO(LOGGER, "Z: %f", transform.transform.translation.z);
    // Pick the object
    pick_and_place_class.pickObject(transform.transform);

    pick_and_place_class.placeObject();
  }

  
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 