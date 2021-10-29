#!/bin/bash
xterm -geometry 80x5+0+0   -e "/opt/ros/melodic/bin/roslaunch happymimi_bringup minimal.launch" &
sleep 5s
xterm -geometry 80x5+0+130 -e "/opt/ros/melodic/bin/roslaunch happymimi_bringup sensor.launch" &
sleep 5s
xterm -geometry 80x5+0+260 -e "~/catkin_ws/devel/lib/chaser19/chaser19" &
sleep 5s
xterm -geometry 80x5+0+360 -e "/opt/ros/melodic/bin/rostopic pub /follow_human std_msgs/String start" &
sleep 1s
#xterm -geometry 80x5+0+460 -e "/opt/ros/kinetic/bin/rosbag record -a" &
#sleep 5s
