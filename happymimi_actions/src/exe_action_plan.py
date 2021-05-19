#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionPlanから行動を決定して実行するアクションサーバー
# Author: Issei Iida
# Date: 2020/02/07
#-----------------------------------------------------------------------------

import sys

import rospy
import rosparam
import actionlib
import smach
from smach import StateMachine
import smach_ros
from smach_ros import ActionServerWrapper
from std_msgs.msg import String
from mimi_common_pkg.srv import ManipulateSrv, RecognizeCount
from mimi_common_pkg.msg import ExeActionPlanAction

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import speak, searchLocationName, m6Control 
from common_action_client import (navigationAC, localizeObjectAC, approachPersonAC)


class DecideAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['move', 'mani', 'search', 
                                               'speak', 'all_finish'],
                             input_keys = ['goal_in', 'a_num_in', 'result_message'],
                             output_keys = ['a_action_out', 'a_data_out',
                                            'a_num_out', 'result_message'])
        # Param
        self.action_state = rosparam.get_param('/action_state')

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_ACTION')
        a_count = userdata.a_num_in
        a_plan = userdata.goal_in
        print str(a_count)
        print a_plan
        if a_count < len(a_plan.action):
            a_name = a_plan.action[a_count]
            userdata.a_action_out = a_name
            userdata.a_data_out = a_plan.data[a_count]
            return self.action_state[a_name]
        else:
            rospy.loginfo('all success')
            userdata.a_num_out = 0
            userdata.result_message.data = 'success'
            return 'all_finish'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['move_finish', 'move_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])
        # Publisher
        self.pub_location = rospy.Publisher('/current_location', String, queue_size = 1)

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        speak('Move to ' + data)
        if name == 'go':
            coord_list = searchLocationName(data)
            result = navigationAC(coord_list)
            self.pub_location.publish(data)
        elif name == 'approach':
            result = approachPersonAC()
        else:
            # speak('okey dokey')
            return 'move_finish'
        if result:
            userdata.a_num_out = a_count + 1 
            return 'move_finish'
        else:
            userdata.a_num_out = 0 
            return 'move_failed'


class Mani(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['mani_finish', 'mani_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])
        # Service
        self.grasp_srv = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', ManipulateSrv)
        # Param
        self.object_dict = rosparam.get_param('/object_mapping')

    def execute(self, userdata):
        rospy.loginfo('Executing state: MANI')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        if name == 'grasp':
            obj = self.object_dict[data]
            result = self.grasp_srv(obj).result
        elif name == 'place':
            result = self.arm_srv('place').result
        elif name == 'give':
            m6Control(0.3)
            # speak('Here you are')
            result = self.arm_srv('give').result
        else:
            # speak('Oh my god')
            return 'mani_finish'
        if result:
            userdata.a_num_out = a_count + 1 
            return 'mani_finish'
        else:
            userdata.a_num_out = 0 
            return 'mani_failed'


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['search_finish', 'search_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out', 'obj_num_out'])
        # Service
        self.obj_count_srv = rospy.ServiceProxy('/object/recognize',RecognizeCount)
        # Param
        self.obj_map = rosparam.get_param('/object_mapping')

    def execute(self, userdata):
        rospy.loginfo('Executing state: SEARCH')
        a_count = userdata.num_in
        data = userdata.data_in
        obj = self.obj_map[data]
        m6Control(-0.4)
        rospy.sleep(1.5)
        speak('I search ' + data)
        obj_num = self.obj_count_srv(obj).num
        result = bool(obj_num)
        if result:
            speak("I found " + str(obj_num) + data)
            userdata.a_num_out = a_count + 1 
            userdata.obj_num_out = str(obj_num)
            return 'search_finish'
        else:
            speak("I could't find " + data)
            userdata.a_num_out = 0 
            return 'search_failed'


class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['speak_finish', 'speak_failed'],
                             input_keys = ['action_in', 'data_in',
                                           'num_in', 'obj_num_in'],
                             output_keys = ['a_num_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SPEAK')
        a_count = userdata.num_in
        data = userdata.data_in
        obj_num = userdata.obj_num_in
        if obj_num == 'one':
            return 'speak_finish'
        speak(data)
        userdata.a_num_out = a_count + 1 
        return 'speak_finish'


def main():
    sm_top = StateMachine(outcomes = ['success', 'action_failed', 'preempted'],
                          input_keys = ['goal_message', 'result_message'],
                          output_keys = ['result_message'])
    sm_top.userdata.action_num = 0
    sm_top.userdata.obj_num = 'none'
    with sm_top:
        StateMachine.add('DECIDE_ACTION', DecideAction(),
                         transitions = {'move':'MOVE',
                                        'mani':'MANI',
                                        'search':'SEARCH',
                                        'speak':'SPEAK',
                                        'all_finish':'success'},
                         remapping = {'goal_in':'goal_message',
                                      'a_num_in':'action_num',
                                      'a_action_out':'action_name',
                                      'a_data_out':'data_name',
                                      'a_num_out':'action_num',
                                      'result_out':'result_message'})

        StateMachine.add('MOVE', Move(),
                         transitions = {'move_finish':'DECIDE_ACTION',
                                        'move_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num'})

        StateMachine.add('MANI', Mani(),
                         transitions = {'mani_finish':'DECIDE_ACTION',
                                        'mani_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num'})

        StateMachine.add('SEARCH', Search(),
                         transitions = {'search_finish':'DECIDE_ACTION',
                                        'search_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num',
                                      'obj_num_out':'obj_num'})

        StateMachine.add('SPEAK', Speak(),
                         transitions = {'speak_finish':'DECIDE_ACTION',
                                        'speak_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num',
                                      'obj_num_in':'obj_num'})

    asw = ActionServerWrapper('exe_action_plan', ExeActionPlanAction,
                              wrapped_container = sm_top,
                              succeeded_outcomes = ['success'],
                              aborted_outcomes = ['action_failed'],
                              preempted_outcomes = ['preempted'],
                              goal_key = 'goal_message',
                              result_key = 'result_message')

    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('exe_action_plan', anonymous = True)
    main()
