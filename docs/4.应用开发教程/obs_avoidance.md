# Obstacle Avoidance in Outdoor Environment

The obstacle avoidance software is prepared under the workspace ~/src/uav_space/catkinws_avoidance. We have tested the code thoroughly during factory to ensure the complete suite works properly. The supported OS is Ubuntu 18.04 with ROS melodic. The avoidance package is delivered in the dev_XDKJYuav branch, so be sure to switch to that branch to avoid potential software bugs.

```
# open a terminal (terminal 1)
cd ~/src/uav_space/catkinws_avoidance
source devel/setup.bash
# add environment setting
. ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
roslaunch local_planner local_planner_stereo.launch

# open another terminal to view image (terminal 2)
cd ~/src/uav_space/catkinws_avoidance
source devel/setup.bash
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color

# open another terminal (terminal 3)
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```
