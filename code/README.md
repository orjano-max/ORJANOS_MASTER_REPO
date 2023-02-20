
# ROS Information

A full tutorial on how to set up the husky has been made: https://mas514-husky_group.gitlab.io/mas-514-husky/ 


This repo also contains packages used to run the husky robot.

install slam_toolbox and nav2 before using this package.

Download the repo into your own workspace:

mkdir husky_project_ws/src

From the workspace directory, use colcon build

run the husky with lidar and imu using the following command:

ros2 launch husky_group husky1.launch.py

Launch the navigation by following our tutorial:

url: https://mas514-husky_group.gitlab.io/mas-514-husky/src/Part%204%20Localization%20and%20Mapping.html



