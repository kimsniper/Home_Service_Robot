#!/bin/sh
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot world.launch " &
sleep 15
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot amcl.launch " & 
sleep 5
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot view_navigation.launch " &
sleep 5
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot add_markers.launch " &
sleep 3
xterm  -e  " source ../../devel/setup.bash && rosrun pick_objects pick_objects "
