#!/bin/sh

xterm -e "source /home/workspace/catkin_ws/devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/Building.world" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash && roslaunch gmapping gmapping_demo.launch " &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
