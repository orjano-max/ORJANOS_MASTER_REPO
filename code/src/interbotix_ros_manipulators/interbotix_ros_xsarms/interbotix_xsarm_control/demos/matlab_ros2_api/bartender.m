close all;

% This script makes the end-effector perform pick, pour, and place tasks
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
% Then change to this directory in your MATLAB console and type 'bartender'

rosshutdown

bot = InterbotixManipulatorXS("wx250s", "arm", "gripper");
bot.arm.set_ee_pose_components(x=0.3, z=0.2);
bot.arm.set_single_joint_position("waist", pi/2.0);
bot.gripper.open();
bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16);
bot.gripper.close();
bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16);
bot.arm.set_single_joint_position("waist", -pi/2.0);
bot.arm.set_ee_cartesian_trajectory(pitch=1.5);
bot.arm.set_ee_cartesian_trajectory(pitch=-1.5);
bot.arm.set_single_joint_position("waist", pi/2.0);
bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16);
bot.gripper.open();
bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16);
bot.arm.go_to_home_pose();
bot.arm.go_to_sleep_pose();

rosshutdown
bot.stop_timers();
