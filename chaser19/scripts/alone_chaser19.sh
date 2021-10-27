#!/bin/bash
xterm -geometry 80x5+0+0   -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 10s
xterm -geometry 80x5+0+130 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 5s
xterm -geometry 80x5+0+260 -e "/opt/ros/kinetic/bin/rqt -s kobuki_dashboard" &
sleep 5s
xterm -geometry 80x5+0+360 -e "~/catkin_ws/devel/lib/chaser19/chaser19" &
sleep 5s
xterm -geometry 80x5+0+910 -e "/opt/ros/kinetic/bin/rostopic pub /follow_human std_msgs/String start" &
sleep 1s
#xterm -geometry 80x5+0+460 -e "/opt/ros/kinetic/bin/rosbag record -a" &
#sleep 5s
