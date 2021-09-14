#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 目的地の名前と座標を設定するサービスサーバー
# Author: Issei Iida
#-----------------------------------------------------------
import os
import subprocess as sp
import rospy
import roslib.packages
from location_setup.srv import LocationSetup, LocationSetupResponse
from locsetup import AddLoc, SaveLoc

map_path = roslib.packages.get_pkg_dir("happymimi_navigation")


def callback_srv(srv_req):
    if srv_req.state == 'add':
        rospy.loginfo("----Start Add State----")
        addLoc = AddLoc()
        return LocationSetupResponse(result = addLoc.execute(srv_req.loc_name))
    elif srv_req.state == 'save':
        rospy.loginfo("----Start Save State----")
        saveLoc = SaveLoc()
        merge_file_name = "{0}_map.yaml".format(srv_req.file_name)
        merge_path = os.path.join(map_path, "maps", merge_file_name)
        # sp.Popen(['rosrun','map_server','map_saver','-f', merge_path])
        rospy.loginfo("The map was saved with the name <" + srv_req.file_name + ">")
        return LocationSetupResponse(result = saveLoc.execute(srv_req.file_name))
    else:
        rospy.logerr("<" + srv_req.state + "> is an incorrect value. Please re-enter.")
        return LocationSetupResponse(result = False)

if __name__ == '__main__':
    rospy.init_node('create_location', anonymous = True)
    srv = rospy.Service('/create_location', LocationSetup, callback_srv)
    rospy.loginfo("Ready to create_location_server")
    rospy.spin()
