#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するServiceServer
# Author: Yusuke Kanazawa
# Data: 2021/09/17
# Memo: ドアが閉まっていることを前提条件とする
# サービスで「進む距離」と「速度」を指定できる
#-------------------------------------------------------------
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from happymimi_teleop.srv import EnterRoomGen2, EnterRoomGen2Response

# 速度と進む距離を取得
# 
class EnterRoomServer():
    def __init__(self):
        service = rospy.Service('/enter_room_server', EnterRoomGen2, self.execute)
        rospy.loginfo("Ready to set enter_room_server")
        # Publisher
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        # Value
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self, srv_req):
        # Value
        vel = Twist()
        vel.linear.x = srv_req.velocity
        target_dist = srv_req.distance
        safe_dist = 1.0
        target_time = target_dist / vel.linear.x # 速度と進行距離から実行時間を計算
        start_time = rospy.get_time() # 実行開始時間を格納

        while not rospy.is_shutdown():
            if self.front_laser_dist >= safe_dist and (rospy.get_time() - start_time) <= target_time: # 安全距離かつ実行時間内のとき実行
                # print "now_time = ", rospy.get_time() - start_time #経過時間確認用
                self.twist_pub.publish(vel)
            elif self.front_laser_dist <= safe_dist: # 障害物があるとき実行
                print "Please open the door"
                rospy.sleep(3.0)
            else: # megaroverを停止して、終了時刻を格納してbreak
                vel.linear.x = 0.0
                self.twist_pub.publish(vel)
                finish_time = rospy.get_time() - start_time
                break
            
        if finish_time >= target_time:
            print "enter_room finish [distance:", srv_req.distance, "velocity:", srv_req.velocity, "]"
            return EnterRoomGen2Response(result = True)
        else:
            return EnterRoomGen2Response(result = False)

if __name__ == '__main__':
    rospy.init_node('enter_room')
    ers = EnterRoomServer()
    rospy.spin()
