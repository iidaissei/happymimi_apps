#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するServiceServer
# Author: Issei Iida
# Date: 2021/05/28
# Memo: ドアが閉まっていることを前提条件とする
#--------------------------------------------------------------------
import rospy
from sensor_msgs.msg import LaserScan
from base_control import BaseControl
from happymimi_teleop.srv import EnterRoom, EnterRoomResponse

class EnterRoomServer():
    def __init__(self):
        service = rospy.Service('/enter_room_server', EnterRoom, self.execute)
        rospy.loginfo("Ready to set enter_room_server")
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        # Value
        self.front_laser_dist = 999.9
        self.bc = BaseControl()

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self, srv_req):
        try:
            # target_dist = self.front_laser_dist + srv_req.dist
            target_dist = 0.3 + srv_req.dist
            # speak("Please open the door")
            print("Please open the door")
            while self.front_laser_dist <= target_dist:
                rospy.sleep(0.5)
            # speak("Thank you")
            print("Thank you")
            self.bc.translateDist(target_dist)
            return EnterRoomResponse(result = True)
        except rospy.ROSInterruptException:
            rospy.logerr("!!Interrupted!!")
            return EnterRoomResponse(result = False)

if __name__ == '__main__':
    rospy.init_node('enter_room', anonymous = True)
    ers = EnterRoomServer()
    rospy.spin()
