#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: ロケーションファイルの設定を行うサービスサーバー
# Author: Issei Iida
#-----------------------------------------------------------
import yaml
import rospy
import rosparam
import roslib.packages
from location_setup.srv import LocationSetup, LocationSetupResponse
from locsetup import AddLoc, SaveLoc


def callback_srv(srv_req):
    param_path = roslib.packages.get_pkg_dir("happymimi_params")
    merge_file_name = "{0}_loc.yaml".format(srv_req.file_name)
    with open(os.path.join(param_path, "param/location", merge_file_name)) as target_file:
        yml = yaml.load(target_file)
    addLoc = AddLoc()
    saveLoc = SaveLoc()
    if srv_req.state == 'add':
        rospy.loginfo("----Start Add State----")
        yml.update(addLoc.createLocDict(srv_req.loc_name))
        return LocationSetupResponse(result = saveLoc.execute(srv_req.file_name, yml))
    elif srv_req.state == 'remove':
        rospy.loginfo("----Start Remove State----")
        del yml[srv_req.loc_name]
        return LocationSetupResponse(result = saveLoc.execute(srv_req.file_name, yml))
    elif srv_req.state == 'change_name':
        rospy.loginfo("----Start Change_Name State----")
        key_list = srv_req.loc_name.split(',')
        yml[key_list[1]] = yml.pop(key_list[0])
        return LocationSetupResponse(result = saveLoc.execute(srv_req.file_name, yml))
    elif srv_req.state == 'change_coord':
        rospy.loginfo("----Start Change_Coord State----")
        new_coord = addLoc.createLocDict(srv_req.loc_name)
        yml[srv_req.loc_name] = new_coord
        return LocationSetupResponse(result = True)
    # elif srv_req.state == 'save':
    #     rospy.loginfo("----Start Save State----")
    #     saveLoc = SaveLoc()
    #     rospy.loginfo("The map was saved with the name <" + srv_req.file_name + ">")
    #     return LocationSetupResponse(result = saveLoc.execute(srv_req.file_name))
    else:
        rospy.logerr("<" + srv_req.state + "> is an incorrect value. Please re-enter.")
        return LocationSetupResponse(result = False)


if __name__ == '__main__':
    rospy.init_node('edit_location', anonymous = True)
    srv = rospy.Service('/edit_location', LocationSetup, callback_srv)
    rospy.loginfo("Ready to edit_location_server")
    rospy.spin()
