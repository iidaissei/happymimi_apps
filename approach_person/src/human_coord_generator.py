#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import roslib
import tf2_ros
import tf
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
        # Service
        self.ml_srv = rospy.ServiceProxy('/recognition/multiple_localize', MultipleLocalize)
        self.ghc = GenerateHumanCoord()
        self.human_coord_dict = {}
        self.map_range = rospy.get_param('/map_range')

    def saveDict(self):
        param_path = roslib.packages.get_pkg_dir("happymimi_params")
        rospy.set_param('/tmp_human_location', self.human_coord_dict)
        rosparam.dump_params(param_path + '/location/'  + 'tmp_human_location.yaml', '/tmp_human_location')

    def judgeMapin(self, coord):
        rpy = tf.transformations.euler_from_quaternion((coord[0], coord[1], coord[2], coord[3]))
        print "AAAAAA"
        print rpy
        if rpy[0] < self.map_range["min_x"] or rpy[0] > self.map_range["max_x"]:
            jm_result = False
        elif rpy[1] < self.map_range["min_y"] or rpy[0] > self.map_range["max_y"]:
            jm_result = False
        else:
            jm_result = True
        return jm_result

    def execute(self, srv_req):
        dist_data = self.ml_srv(target_name = "person")
        dist_list = list(dist_data.points)
        list_len  = len(dist_list)
        if list_len <= 1:
            return SimpleTrgResponse(result = False)
        else:
            print dist_data.points
            for i in range(len(dist_list)):
                frame_id = "human_" + str(i)
                # map座標系に変換してlocation dictを作成
                human_dict = self.ghc.execute(frame_id, dist_data.points[i].x, dist_data.points[i].y)
                if self.judgeMapin(human_dict[frame_id]):
                    self.human_coord_dict.update(human_dict)
                else:
                    pass
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
