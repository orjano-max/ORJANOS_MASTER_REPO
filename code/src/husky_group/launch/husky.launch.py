import os
from symbol import parameters
import math
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('use_manipulator', default_value='false',
                          description='Wether or not to use the interbotix manipulator'),
]

os.environ["HUSKY_TOP_PLATE_ENABLED"] = "false"
os.environ["HUSKY_IMU_XYZ"] = "0 0 0"
os.environ["HUSKY_IMU_RPY"] = "0 0 0"

def generate_launch_description():
    
    #Declare "use_manipulator" argument
    use_manipulator = LaunchConfiguration('use_manipulator')

    # Get LIDAR parameters
    lidar_params = PathJoinSubstitution(
        [FindPackageShare('husky_group'),
        'params',
        'ouster_lidar.yaml'],
    )

    localization_params = PathJoinSubstitution(
        [FindPackageShare('husky_group'),
        'params',
        'localization.yaml'],
    )

    husky_urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_group"), "urdf", "husky_urdf_extras.urdf"]
                )
    
    interbotix_urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_group"), "urdf", "interbotix_urdf_extras.urdf"]
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
            "urdf_extras:=",husky_urdf_extras_path
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_group"),
        "params",
        "control.yaml"],
    )

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
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["husky_velocity_controller"],
        output="screen",
    )


    #Launch husky_control
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control_launch.py'])),
        launch_arguments ={
            "params_file" : localization_params
        }.items()
    )

    #Launch husky_control
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_group"), 'launch', 'teleop_base_launch.py'])),
    )

    #Launch husky_control
    launch_husky_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_group"), 'launch', 'teleop_joy_launch.py'])),
    )


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

    GroupAction(
        condition=IfCondition(use_manipulator)
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

    launch_scene_geometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('scene_geometry_publisher'), 'launch', 'scene_geometry_publisher_launch.py'])),
            launch_arguments ={
            'robot_model' : 'vx300',
            }.items()
    )

    """  # Static transform that places the interbotics robot on the husky
    node_tf_realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["0", "0", "0.05", "0", "0", "0", "vx300/ee_gripper_link", "camera_link"],
    ) """
    


    ld = LaunchDescription()
    
    # Launch Interbotix manipulator
    #ld.add_action(node_tf_realsense)
    ld.add_action(node_tf_manipulator)
    ld.add_action(launch_interbotix_moveit)

    # Launch pointcloud to laserscan, imu and lidar
    ld.add_action(node_pointcloud_to_laserscan)
    ld.add_action(node_um7_imu)
    ld.add_action(launch_ouster_lidar)

    # Launch Husky UGV
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_husky_velocity_controller)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)
    ld.add_action(launch_husky_teleop_joy)
    
    # Add collision objects to planning scene
    ld.add_action(launch_scene_geometry_publisher)
    
    return ld

