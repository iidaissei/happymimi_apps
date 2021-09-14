#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: ロケーション辞書データを生成するモジュール
#-----------------------------------------------------------
import os
import rospy
import roslib
import rosparam
from tf import TransformListener

# map_frame = rosparam.get_param("/happymimi/map_frame")
# base_link_frame = rosparam.get_param("/happymimi/base_link_frame")
param_path = roslib.packages.get_pkg_dir("happymimi_params")


def checkName(name, search_target):
    if name in search_target:
        rospy.logerr(name + " has been registerd. Please enter a different name.")
        return False
    elif name == '':
        rospy.logerr("No location name enterd.")
        return False
    else:
        return True


class AddLoc:
    loc_dict = {}
    def __init__(self):
        self.tf = TransformListener()

    def createLocDict(self, loc_name):
        cur_loc_dict = {}
        position, rotation = self.tf.lookupTransform("/map", "/base_link", rospy.Time(0))
        cur_loc_dict[loc_name] = [position[0], position[1], rotation[2], rotation[3]]
        # position, rotation = tf.lookupTransform(map_frame, base_link_frame, rospy.Time(0))
        # cur_loc_dict[loc_name] = [1, 2, 3, 4]
        return cur_loc_dict

    def execute(self, name):
        if checkName(name, self.loc_dict):
            self.loc_dict.update(self.createLocDict(name))
            print self.loc_dict
            rospy.loginfo("Successful registration of " + name)
            return True
        else:
            return False


class SaveLoc:
    def execute(self, file_name, loc_dict=None):
        if loc_dict is None:
            loc_dict = AddLoc.loc_dict
        rospy.set_param('/location_dict', loc_dict)
        merge_file_name = "{0}_loc.yaml".format(file_name)
        merge_path = os.path.join(param_path, "param/location", merge_file_name)
        rosparam.dump_params(merge_path, '/location_dict')
        print rosparam.get_param('/location_dict')
        rospy.loginfo("The location was saved successfully!")
        AddLoc.loc_dict = {}
        return True

