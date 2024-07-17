#!/bin/sh
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot world.launch " &
sleep 15
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot mapping.launch " & 
sleep 5
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot view_navigation.launch " &
sleep 5
xterm  -e  " source ../../devel/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py "
