#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: 足回り制御を行うPythonモジュール
# Author: Issei Iida
# Date: 2021/05/19
#--------------------------------------------------------------------
import time
import math
import rospy
from geometry_msgs.msg import Twist

class BaseControl():
    def __init__(self):
        # Publisher
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # Value
        self.twist_value = Twist()
        self.target_time = 0.0
        self.rate = rospy.Rate(30)

    def publishTwist(self):
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.twist_pub.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.linear.x = 0.0
        self.twist_value.angular.z = 0.0
        self.pub_twist.publish(self.twist_value)

    def translateDist(self, distance, speed = 0.2):
        self.target_time = abs(distance / speed)
        self.twist_value.linear.x = speed
        self.twist_value.angular.z = 0.0
        self.publishTwist()

    def rotateAngle(self, degree, speed = 0.2):
        self.target_time = (math.radians(degree) / speed)
        self.twist_value.linear.x = 0.0
        self.twist_value.angular.z = speed
        self.publishTwist()
