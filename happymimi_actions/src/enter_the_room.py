#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するActionServer
# Author: Issei Iida
# Date: 2021/05/18
# Memo: ドアが閉まっていることを前提条件とする
#--------------------------------------------------------------------

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from happymimi_actions.msg import EnterTheRoomAction, EnterTheRoomResult
from gcp_texttospeech.srv import TTS

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import BaseCarrier


class EnterTheRoomAS():
    def __init__(self):
        # ActionServer
        self.sas = actionlib.SimpleActionServer('enter_the_room', EnterTheRoomAction,
                                                execute_cb = self.execute,
                                                auto_start = False)
        self.sas.start()
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        # Service
        self.tts_srv = rospy.ServiceProxy('/tts', TTS)
        # Value
        self.bc = BaseCarrier()
        self.result = EnterTheRoomResult()
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def detection(self, receive_msg):
        # -0.05は理論値と実験値の誤差を加味した値
        target_distance = self.front_laser_dist + receive_msg - 0.05
        self.tts_srv("Please open the door")
        while self.front_laser_dist <= target_distance:
            rospy.sleep(0.1)
        self.tts_srv("Thank you")
        return target_distance

    def execute(self, goal):
        try:
            rospy.loginfo("Start EnterTheRoom")
            distance = self.detection(goal.distance)
            self.bc.translateDist(distance)
            self.result.data = True
            self.sas.set_succeeded(self.result)
            rospy.loginfo("Finish EnterTheRoom")
        except rospy.ROSInterruptException:
            rospy.logwarn('**Interrupted**')
            pass


if __name__ == '__main__':
    rospy.init_node('enter_the_room', anonymous = True)
    eras = EnterTheRoomAS()
    rospy.spin()
