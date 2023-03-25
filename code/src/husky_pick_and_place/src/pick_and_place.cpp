// This file is the executable in the husky_pick_and_place node.
// It contains an implementation of the husky_pick_and_place class,
// where a scene is read from a .scene file contained in the "params" folder of
// the husky_pick_and_place ros package.
//
// Author: Ørjan Øvsthus

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PickAndPlace
{
  public:

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper_;

    PickAndPlace(rclcpp::Node::SharedPtr node, std::string PLANNING_GROUP_ARM = "interbotix_arm", std::string PLANNING_GROUP_GRIPPER = "interbotix_gripper")
      : node_(node), PLANNING_GROUP_ARM_(PLANNING_GROUP_ARM), PLANNING_GROUP_GRIPPER_(PLANNING_GROUP_GRIPPER)
      {
        
        // We spin up a SingleThreadedExecutor for the current state monitor to get information
        // about the robot's state.
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        std::thread([&executor]() { executor.spin(); }).detach();

        move_group_interface_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_ARM_);
        move_group_interface_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_GRIPPER_);


      }
    
    
    

    void initalizeMoveGroup()
    {

    }

    void pickObject()
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    
      // 1. Move to home position
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



      // 2. Place the TCP (Tool Center Point, the tip of the robot) on top of the thingy 
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
      
      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.x = qNew.getX();
      target_pose1.orientation.y = qNew.getY();
      target_pose1.orientation.z = qNew.getZ();
      target_pose1.orientation.w = qNew.getW();
      target_pose1.position.x = current_pose.pose.position.x + 0.2;
      target_pose1.position.y = current_pose.pose.position.y;
      target_pose1.position.z = current_pose.pose.position.z - 0.2;
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

      // 3. Place the TCP (Tool Center Point, the tip of the robot) on the thingy 
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

      // 6. Release the thingy
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


      // 7. Move to sleep position
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


  private:
    rclcpp::Node::SharedPtr node_;
    std::string PLANNING_GROUP_ARM_;
    std::string PLANNING_GROUP_GRIPPER_;
    
    
    

};




static const rclcpp::Logger LOGGER = rclcpp::get_logger("husky_pick_and_place");

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  static const std::string manipulator_namespace = "vx300";

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("pick_and_place_node", manipulator_namespace, options);

  // Initiating the pick and place class
  PickAndPlace pick_and_place_class = PickAndPlace(node);
  
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", pick_and_place_class.move_group_interface_arm_->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", pick_and_place_class.move_group_interface_arm_->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(pick_and_place_class.move_group_interface_arm_->getJointModelGroupNames().begin(), pick_and_place_class.move_group_interface_arm_->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Pick the object
  pick_and_place_class.pickObject();
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
} 