import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
          
    # Launch Realsense camera
    launch_realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']))
    )

    # Launch continuous detection
    launch_apriltag_continuous_detection = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('husky_interbotix'), 'launch', 'continuous_detection.launch.xml'])),
    )


    ld = LaunchDescription()

    # Launch Realsense camera
    ld.add_action(launch_realsense_camera)

    # Launch Apriltag
    ld.add_action(launch_apriltag_continuous_detection)

    return ld