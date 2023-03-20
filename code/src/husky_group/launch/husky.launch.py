from symbol import parameters
import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

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

    urdf_extras_path = PathJoinSubstitution(
                [FindPackageShare("husky_group"), "urdf", "husky_urdf_extras.urdf"]
    )

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_group"),
        "params",
        "control.yaml"],
    )
    
    # Launch the husky robot using the husky_uia uia_master_husky repo
    launch_husky_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_base"), 'launch', 'base_launch.py'])),
        launch_arguments ={
            "urdf_extras" : urdf_extras_path,
            "localization_params" : localization_params,
            "config_husky_velocity_controller" : config_husky_velocity_controller,
        }.items()
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


    ld = LaunchDescription()

    # Launch pointcloud to laserscan, imu and lidar
    ld.add_action(node_pointcloud_to_laserscan)
    ld.add_action(node_um7_imu)
    ld.add_action(launch_ouster_lidar)

    # Launch Husky UGV
    ld.add_action(launch_husky_base)

    return ld

