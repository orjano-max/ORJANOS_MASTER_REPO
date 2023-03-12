import os
from symbol import parameters
import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]

os.environ["HUSKY_TOP_PLATE_ENABLED"] = "false"

def generate_launch_description():
    
    urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_group"), "urdf", "husky_urdf_extras.urdf"]
                )

    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "urdf_extras:=", urdf_extras_path,
            " ",
            "gazebo_controllers:=", config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=['-entity',
                   'husky',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    launch_interbotix = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'hardware_type' : 'fake'
            }.items()
    )

    # Static transform that places the interbotics robot on the husky
    node_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["-0.44", "-0.104", "0", "-1.5708", "0", "0", "user_rail_link", "world"],
    )

    launch_scene_geometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('scene_geometry_publisher'), 'launch', 'scene_geometry_publisher_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'hardware_type' : 'fake'
            }.items()
    )

    

    ld = LaunchDescription(ARGUMENTS)
    # Launch Interbotix manipulator
    ld.add_action(launch_interbotix)
    ld.add_action(node_tf_publisher)
    ld.add_action(launch_scene_geometry_publisher)

    # Launch husky
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)

    


    return ld