import os
from symbol import parameters
import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

os.environ["HUSKY_TOP_PLATE_ENABLED"] = "false"

def generate_launch_description():
    # Get parameters
    params = PathJoinSubstitution(
        [FindPackageShare('husky_group'),
        'params',
        'params.yaml'],
    )

    # Get LIDAR parameters
    lidar_params = PathJoinSubstitution(
        [FindPackageShare('husky_group'),
        'params',
        'ouster_lidar.yaml'],
    )

    urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_group"), "urdf", "husky_urdf_extras.urdf"]
                )
    
    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
        "config",
        "control.yaml"],
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
            "urdf_extras:=",urdf_extras_path
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_husky_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["husky_velocity_controller"],
        output="screen",
    )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))
    
    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))

    # Launch husky_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_husky_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_joy.launch.py'])))


    # Launch husky_bringup/accessories.launch.py which is the sensors commonly used on the Husky.
    launch_husky_accessories = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_bringup"), 'launch', 'accessories.launch.py'])))



    #  ----------------------- ALT UNDER HER VAR HER FRA F??R----------------------------------


    #Launch the UM7 IMU
    node_um7_imu = Node(
        package="um7",
        executable="um7_node",
        output="screen",
    )

    #Launch the LIDAR
    launch_ouster_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("ros2_ouster"), 'launch', 'os1_launch.py'])),
        launch_arguments ={
            "params_file" : lidar_params
        }.items()
    )


    #Launch the pointcloud to laserscan package
    node_pointcloud_to_laserscan =  Node(
    	name="pointcloud_to_laserscan",
        package="pointcloud_to_laserscan",
        remappings=[('cloud_in', '/points')],
        executable="pointcloud_to_laserscan_node",
        parameters=[{
        'target_frame': 'base_laser',
        'transform_tolerance': 0.01,
        'min_height': -0.50,
        'max_height': 0.2,
        'angle_min': -math.pi,  # -M_PI/2
        'angle_max': math.pi,  # M_PI/2
        'angle_increment': 0.0087,  # M_PI/360.0
        'scan_time': 0.3333,
        'range_min': 0.8,
        'range_max': 120.0,
        'use_inf': True,
        'inf_epsilon': 1.0,
        }]
    )

    launch_interbotix_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_control'), 'launch', 'xsarm_control.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'use_rviz' : 'false',
            }.items()
    )

    launch_interbotix_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            'use_moveit_rviz' : 'false',
            }.items()
    )

    # Static transform that places the interbotics robot on the husky
    node_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["-0.44", "-0.104", "0", "-1.5708", "0", "0", "user_rail_link", "world"],
    )
    


    ld = LaunchDescription()
    
    ld.add_action(node_pointcloud_to_laserscan)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_husky_velocity_controller)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)
    ld.add_action(launch_husky_teleop_joy)
    ld.add_action(launch_husky_accessories)
    #ld.add_action(node_um7_imu)
    ld.add_action(launch_ouster_lidar)
    #ld.add_action(launch_interbotix_control)
    ld.add_action(launch_interbotix_moveit)
    ld.add_action(node_tf_publisher)
    

    return ld

