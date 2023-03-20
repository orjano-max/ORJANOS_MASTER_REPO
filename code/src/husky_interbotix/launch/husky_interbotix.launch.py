from symbol import parameters
import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     
    interbotix_urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_interbotix"), "urdf", "interbotix_urdf_extras.urdf"]
    )
     
    # Launch the husky robot using the husky_uia uia_master_husky repo
    launch_husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_group"), 'launch', 'husky.launch.py'])),
    )

    launch_interbotix_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'use_moveit_rviz' : 'false',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            }.items()
    )
     

    # Static transform that places the interbotics robot on the husky
    node_tf_manipulator = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["-0.44", "-0.104", "0", "-1.5708", "0", "0", "user_rail_link", "world"],
    )

    # Publish scene geometry
    launch_scene_geometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('scene_geometry_publisher'), 'launch', 'scene_geometry_publisher_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            }.items()
    )


    ld = LaunchDescription()

    # Launch Interbotix arm with moveit
    ld.add_action(launch_interbotix_moveit)
    ld.add_action(node_tf_manipulator)
    # Publish scene
    ld.add_action(launch_scene_geometry_publisher)

    # Launch husky
    ld.add_action(launch_husky)

    

    return ld

