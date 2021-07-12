#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ジョイスティックで足回り制御をするROSノード
# Author: Issei Iida
# Date: 2021/06/17
# Memo: dualshock4 コントローラに対応したキー配置設定
#--------------------------------------------------------------------
import rospy
import rosparam
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyCtrMegarover():
    def __init__(self):
        # Publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        # Subscriber
        rospy.Subscriber("joy", Joy, self.joyCB)
        # Value
        self.twist = Twist()
        self.linear = 1
        self.angular = 0
        self.safety = 10
        self.l_scale = rospy.get_param("joycon_teleop/joycon/l_scale")
        self.a_scale = rospy.get_param("joycon_teleop/joycon/a_scale")

    def joyCB(self, joy):
        try:
            if joy.buttons[self.safety]:
                self.twist.angular.z = self.a_scale * joy.axes[self.angular]
                self.twist.linear.x = self.l_scale * joy.axes[self.linear]
            else:
                self.twist.angular.z = 0.0
                self.twist.linear.x = 0.0
            self.vel_pub.publish(self.twist)
        except rospy.ROSInterruptException:
            rospy.logerr("!Interrupted!")
            pass

if __name__ == '__main__':
    rospy.init_node("joycon_teleop", anonymous = True)
    jcm = JoyCtrMegarover()
    rospy.spin()
