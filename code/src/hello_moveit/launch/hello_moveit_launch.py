from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()
    moveit_config = (
    MoveItConfigsBuilder("vx300","robot_description"))
    moveit_config.robot_description_kinematics = 

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="hello_moveit",
        package="hello_moveit",
        executable="hello_moveit",w
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])

