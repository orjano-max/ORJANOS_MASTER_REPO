from symbol import parameters
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    interbotix_urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_interbotix"), "urdf", "interbotix_urdf_extras.urdf"]
    )

    interbotix_srdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_interbotix"), "srdf", "interbotix_srdf_extras.srdf"]
    )
    
    
    # Launch the husky robot using the husky_uia uia_master_husky repo
    launch_husky_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_group"), 'launch', 'sim_husky.launch.py'])),
    )
    
    launch_interbotix_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            'use_moveit_rviz' : 'false',
            'hardware_type' : 'fake',
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
            'hardware_type' : 'fake',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            }.items()
    )

    launch_pick_and_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('husky_pick_and_place'), 'launch', 'pick_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'hardware_type' : 'fake',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            }.items()
    )

    launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('husky_interbotix'), 'launch', 'rviz_moveit_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'hardware_type' : 'fake',
            'external_urdf_loc' : interbotix_urdf_extras_path,
            'external_srdf_loc' : interbotix_srdf_extras_path,
            }.items()
    )

    ld = LaunchDescription(ARGUMENTS)    

    # Launch Interbotix arm with moveit
    ld.add_action(launch_interbotix_moveit)
    ld.add_action(node_tf_manipulator)
    # Publish scene
    ld.add_action(launch_scene_geometry_publisher)

    # Launch husky
    ld.add_action(launch_husky_simulation)

    #Launch pick and place listener
    ld.add_action(launch_pick_and_place)

    # Launch rviz
    ld.add_action(launch_rviz)
    
    return ld