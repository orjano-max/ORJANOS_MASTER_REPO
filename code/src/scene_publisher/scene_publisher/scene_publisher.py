import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_py_bindings_tools import moveit_get_python_bindings
from moveit_py_bindings_tools import load_moveit_bindings
from moveit_py_bindings_tools import OBJECT_NAME, PLANNING_SCENE_TOPIC, PLANNING_SCENE_WORLD_TOPIC
from moveit_py_bindings_tools import PyPlanningSceneInterface, PyPlanningSceneMonitor, PyRobotModel
from moveit_py_bindings_tools import PyRobotState, PyJointModelGroup, PyMoveGroupInterface
from moveit_ros_planning_interface import _moveit_roscpp_initializer as moveit_cpp_api

def main(args=None):
    rclpy.init(args=args)

    # Initialize MoveItCpp API
    moveit_cpp_api.MoveItCppInitializer()

    # Create a MoveGroupInterface for the planning group
    move_group = PyMoveGroupInterface("interbotix_arm")

    # Set the target pose
    target_pose = Pose()
    target_pose.position.x = 0.6
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    move_group.set_pose_target(target_pose)

    # Create a plan to the target pose
    plan = move_group.plan()

    # Execute the plan
    if plan and move_group.execute(plan):
        print("Plan executed successfully!")
    else:
        print("Planning failed!")

    # Shutdown ROS
    rclpy.shutdown()

if __name__ == "__main__":
    main()