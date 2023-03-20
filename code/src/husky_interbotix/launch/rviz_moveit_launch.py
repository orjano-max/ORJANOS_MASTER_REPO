
import os

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    show_ar_tag_launch_arg = LaunchConfiguration('show_ar_tag')
    use_world_frame_launch_arg = LaunchConfiguration('use_world_frame')
    external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    rviz_frame = 'base_footprint'

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('husky_interbotix'), 
        'rviz', 
        'rviz_config.rviz'])


    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    robot_description = {'robot_description': robot_description_launch_arg}

    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])

    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model_launch_arg.perform(context),
                config_path=config_path
            ),
    }

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
        'kinematics.yaml',
    ])

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }

    remappings = [
        (
            f'{robot_name_launch_arg.perform(context)}/get_planning_scene',
            f'/{robot_name_launch_arg.perform(context)}/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}/gripper_controller/follow_joint_trajectory'
        ),
    ]

    moveit_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # namespace=robot_name_launch_arg,
        arguments=[
            '-d', rviz_config_file,
            '-f', rviz_frame,
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': use_sim_time_param},
        ],
        remappings=remappings,
        output={'both': 'log'},
    )

    return [moveit_rviz_node]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description=(
                'the file path to the custom semantic description file that you would like to '
                "include in the Interbotix robot's semantic description."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration/",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_common_sim'),
                'worlds',
                'interbotix.world',
            ]),
            description="the file path to the Gazebo 'world' file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
