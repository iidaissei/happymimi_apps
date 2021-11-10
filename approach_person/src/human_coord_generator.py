#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rosparam
import tf
import tf2_ros
import actionlib
import geometry_msgs.msg

from approach_person.msg import PubHumanTFAction, PubHumanTFGoal
from happymimi_msgs.srv import SimpleTrg

class GenerateHumanCoord():
    def __init__(self):
        self.sac = actionlib.SimpleActionClient('pub_human_tf', PubHumanTFAction)
        self.sac.wait_for_server()
        slef.goal = PubHumanTFGoal()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.rate = rospy.Rate(10.0)
        self.human_dict = {}

    def execute(self, frame_name, dist_x, dist_y):
        self.goal.name = frame_name
        self.goal.dist_x = dist_x
        self.goal.dist_y = dist_y
        self.sac.send_goal(self.goal)
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                self.human_dict[frame_name].append(trans.transform.translation.x)
                self.human_dict[frame_name].append(trans.transform.translation.y)
                self.human_dict[frame_name].append(trans.transform.rotation.z)
                self.human_dict[frame_name].append(trans.transform.rotation.w)
                print self.human_dict
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            self.rate.sleep()
        self.sac.cancel_goal()
        return self.human_dict


class HumanCoordGeneratorSrv():
    def __init__(self):
        hcg_srv = rospy.Service('human_coord_generator', SimpleTrg, self.execute)
        rospy.loginfo("Ready to human_coord_generator server")
        self.ghTF = GenerateHumanTF()
        self.ghc = GenerateHumanCoord()
        self.human_coord_dict = {}

    def execute(self):
        dist_data = [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]]
        for i range(dist_data):
            frame_id = "human_" + str(i)
            # map座標系に変換してlocation dictを作成
            human_coord = self.ghTF.execute(frame_id, dist_data[i][0], dist_data[i][1])
            self.human_coord_dict.update(human_coord)
            # dict はパラメータとして保存する

        param_path = roslib.packages.get_pkg_dir("approach_person")
        rospy.set_param('/human_location', self.human_coord_dict)
        rosparam.dump_params(param_path + '/config/'  + 'human_location.yaml', '/human_location')
        return SimpleTrgResponse(result = True)


if __name__ == '__main__':
    rospy.init_node('human_coord_generator')
    try:
        hcgs = HumanCoordGeneratorSrv()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
