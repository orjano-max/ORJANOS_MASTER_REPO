from symbol import parameters
import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get parameters

    '''
    launch_husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('husky_group'), 'launch', 'husky1.launch.py']))
    )
    '''
    launch_husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('husky_gazebo'), 'launch', 'gazebo.launch.py']))
    )

    launch_interbotix = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_descriptions'), 'launch', 'xsarm_description.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'use_rviz' : 'false',
            'use_joint_pub_gui' : 'true'
            }.items()
    )

    #Launch the UM7 IMU
    node_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["-0.3", "0", "0.2", "-1.5708", "0", "0", "base_link", "world"],
    )

    

    ld = LaunchDescription()

    ld.add_action(launch_husky)
    ld.add_action(launch_interbotix)
    ld.add_action(node_tf_publisher)


    return ld