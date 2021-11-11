#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import rosparam
import dynamic_reconfigure.client
from happymimi_msgs.srv import StrTrg, StrTrgResponce
from happymimi_navigation.srv import NaviCoord


class ApproachPersonServer():
    def __init__(self):
        # Service
        self.ap_srv = rospy.Service('approach_person_server', StrTrg, self.execute)
        rospy.loginfo("Ready to approach_person_server")
        self.navi_srv = rospy.ServiceProxy('navi_coord_server', NaviCoord)
        # Dynparam
        self.dwa_c = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        # self.realsense_c = dynamic_reconfigure.client.Client('/move_base/local_costmap/realsense_layer')
        self.human_loc = rosparam.get_param('/tmp_human_location')
        self.human_coord = []

    def setParams(self, switch):
        if switch == 'approach':
            goal_tolerance = {'xy_goal_tolerance':0.5, 'yaw_goal_tolerance':6.2}
            # realsense = {'enabled':False}
        elif switch == 'defalt':
            goal_tolerance = {'xy_goal_tolerance':0.15, 'yaw_goal_tolerance':0.08}
            # realsense = {'enabled':True}
        self.dwa_c.update_configuration(goal_tolerance_p)
        self.realsense_c.update_configuration(realsense_p)
        rospy.sleep(0.5)

    def GetCoord(self, human_name):
        if human_name in self.human_loc:
            self.human_coord = self.human_loc[human_name]
            print self.human_coord
            return self.human_coord
        else:
            rospy.logerr("<" + human_name + "> doesn't exist.")
            return False

    def execute(self, srv_req):
        self.setParams(switch = 'approach')
        if self.GetCoord(srv_req.data):
            self.navi_srv(self.human_coor:d)
        else:
            return StrTrgResponce(result = False)
            pass
        self.setParams(switch = 'defalt')
        return StrTrgResponce(result = True)


if __name__ == '__main__':
    rospy.init_node('approach_person_server')
    main()
