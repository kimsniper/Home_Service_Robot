#!/bin/sh
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot world.launch " &
sleep 15
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot amcl.launch " & 
sleep 5
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot view_navigation.launch " &
sleep 1
xterm  -e  " source ../../devel/setup.bash && roslaunch my_robot add_markers.launch add_marker_simulation:=true "
