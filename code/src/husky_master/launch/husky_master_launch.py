from symbol import parameters
import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     
     

    # Static transform that places the interbotics robot on the husky
    node_husky_master = Node(
        package="husky_master",
        executable="husky_master",
        parameters=[{
        'object' : 'case',
        'dimensions' : [0.15, 0.045, 0.04],
        'pick_loc' : [2.0, 0.03, 1.51]
        }],
    )


    ld = LaunchDescription()

    # Launch Interbotix arm with moveit
    ld.add_action(node_husky_master)

    

    return ld

