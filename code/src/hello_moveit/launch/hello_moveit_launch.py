from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():


    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="hello_moveit",
        package="hello_moveit",
        executable="hello_moveit",w
        output="screen",
        parameters=[
            'robot_description' : 
            robot_description_scemantic : 

        ],
    )

    return LaunchDescription([move_group_demo])

