#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import roslib
import tf2_ros
import rosparam
import actionlib
from geometry_msgs.msg import Point
from happymimi_msgs.srv import SimpleTrg, SimpleTrgResponse
from happymimi_recognition_msgs.srv import MultipleLocalize
from approach_person.msg import PubHumanTFAction, PubHumanTFGoal


class GenerateHumanCoord():
    def __init__(self):
        self.sac = actionlib.SimpleActionClient('pub_human_tf', PubHumanTFAction)
        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Value
        self.sac.wait_for_server()
        self.goal = PubHumanTFGoal()
        self.rate = rospy.Rate(10.0)
        self.human_dict = {}

    def execute(self, frame_name, dist_x, dist_y):
        self.goal.name = frame_name
        self.goal.dist_x = dist_x
        self.goal.dist_y = dist_y
        self.sac.send_goal(self.goal)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                self.human_dict[frame_name] = []
                self.human_dict[frame_name].append(trans.transform.translation.x)
                self.human_dict[frame_name].append(trans.transform.translation.y)
                self.human_dict[frame_name].append(trans.transform.rotation.z)
                self.human_dict[frame_name].append(trans.transform.rotation.w)
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
        self.l10n_srv = rospy.ServiceProxy('/recognition/multiple_localize', MultipleLocalize)
        self.ghc = GenerateHumanCoord()
        self.human_coord_dict = {}

    def saveDict(self):
        param_path = roslib.packages.get_pkg_dir("happymimi_params")
        rospy.set_param('/tmp_human_location', self.human_coord_dict)
        rosparam.dump_params(param_path + '/location/'  + 'tmp_human_location.yaml', '/tmp_human_location')

    def execute(self, srv_req):
        # dist_data = [[0.5, 0.5], [0.7, -0.5], [1.0, 0.0]]
        rospy.sleep(3.0)
        dist_data = self.l10n_srv(target_name = "person")
        dist_list = list(dist_data.points)
        print dist_data.points[0].x
        for i in range(len(dist_list)):
            frame_id = "human_" + str(i)
            # map座標系に変換してlocation dictを作成
            human_coord = self.ghc.execute(frame_id, dist_data.points[i].x, dist_data.points[i].y)
            self.human_coord_dict.update(human_coord)
        self.saveDict()
        print self.human_coord_dict
        return SimpleTrgResponse(result = True)


if __name__ == '__main__':
    rospy.init_node('human_coord_generator')
    try:
        hcgs = HumanCoordGeneratorSrv()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
