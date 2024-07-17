#!/bin/sh
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot world.launch " &
sleep 15
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot amcl.launch " & 
sleep 5
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot view_navigation.launch " &
sleep 5
xterm  -e  " source ../../devel/setup.bash && rosrun pick_objects pick_objects "
