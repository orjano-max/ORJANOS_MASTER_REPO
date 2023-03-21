# ORJANOS_MASTER_REPO
This is the github repo for orjano's master thesis

<p align="center">
<img src="Figures/husky_initiated.png" width=500 >
</p>

## Installation
Open terminal.

Clone this repo

~~~bash
git clone https://github.com/orjano-max/ORJANOS_MASTER_REPO
~~~

Go into cloned git folder

~~~bash
cd ORJANOS_MASTER_REPO
~~~

Update the submodules of this repo, there are alot of them
~~~bash
git submodule uptade --init --recursive
~~~

Run the following commands:

~~~bash
cd code/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
~~~

Go back to the root folder of the git repository

~~~bash
cd ORJANOS_MASTER_REPO
~~~

Run rosdep

~~~bash
rosdep install --from-path code/src --ignore-src --rosdistro galactic -y
~~~

Build

~~~bash
colcon build
~~~
