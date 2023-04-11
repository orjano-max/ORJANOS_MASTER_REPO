import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    interbotix_urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_interbotix"), "urdf", "interbotix_urdf_extras.urdf"]
    )

    interbotix_srdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_interbotix"), "srdf", "interbotix_srdf_extras.srdf"]
    )
          
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

    launch_interbotix_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'use_moveit_rviz' : 'false',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            'show_gripper_fingers' : 'true',
            }.items()
    )
     

    # Static transform that places the interbotics robot on the husky
    node_tf_manipulator = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["-0.44", "-0.104", "0", "-1.5708", "0", "0", "user_rail_link", "world"],
    )

    # Static transform that describes case origin
    node_tf_case = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["0.0", "0.0", "-0.0", "0", "0", "0", "case", "case_link"],
    )

    # Publish scene geometry
    launch_scene_geometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('scene_geometry_publisher'), 'launch', 'scene_geometry_publisher_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            }.items()
    )

    # Publish scene geometry
    launch_pick_and_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('husky_pick_and_place'), 'launch', 'pick_and_place_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            }.items()
    )


    ld = LaunchDescription()

    # Launch Interbotix arm with moveit
    ld.add_action(launch_interbotix_moveit)
    ld.add_action(node_tf_manipulator)
    ld.add_action(node_tf_case)
    # Publish scene
    ld.add_action(launch_scene_geometry_publisher)

    # Launch Realsense camera
    ld.add_action(launch_realsense_camera)

    # Launch Apriltag
    ld.add_action(launch_apriltag_continuous_detection)

    #Launch pick and place node
    ld.add_action(launch_pick_and_place)

    return ld