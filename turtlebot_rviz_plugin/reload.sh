#/bin/bash

killall roslaunch
catkin build turtlebot_rviz_plugin && roslaunch turtlebot_bringup rviz.launch &
roslaunch turtlebot_bringup rviz.launch
