#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''auto_charging ROS Node'''
# license removed for brevity

import roslib; roslib.load_manifest('move_base_msgs')

import sys
import os

import rospy
import time
from oryxbot_msgs.srv import *
from oryxbot_msgs.srv import SetArPose
from oryxbot_msgs.srv import SetRelativeMove
from oryxbot_msgs.srv import SetCharge

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

# Brings in the SimpleActionClient
from actionlib_msgs.msg import *
import actionlib
import move_base_msgs.msg
import tf
class Charge(object):

    ac=MoveBaseAction()
    anchor_x=0.4
    anchor_y=0.0
    anchor_theta=0.0

    def adjust(self,mode=127):
        rospy.wait_for_service('/pose_adjust/adjust_service')
        rospy.wait_for_service('Track')
        try:
            PoseAdjustClient=rospy.ServiceProxy("/pose_adjust/adjust_service",SetArPose)
            trackClient=rospy.ServiceProxy("Track",SetCharge)
            tresp=trackClient(0)
            if tresp.success==False :
                return False

            resp=PoseAdjustClient(True)

            tresp=trackClient(127)
            if tresp.success==0 :
                return False

            return  resp.success
        except rospy.ServiceException, e:
            print "adjust() err"
            trackClient=rospy.ServiceProxy("Track",SetCharge)
            tresp=trackClient(127)
            return  False

    def backoff(self):
        rospy.wait_for_service('relative_move')
        try:
            RelativeMoveClient = rospy.ServiceProxy("relative_move",SetRelativeMove)
            relative_move   =   0.19
            mode    =   0
            resp=RelativeMoveClient(relative_move,mode)
            print   "charging...."
            return  resp.success
        except rospy.ServiceException, e:
            print "relative_move is err"
            return  False

    def nav2anchor(self):
        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id="map"
        goal.target_pose.pose.position.x=self.anchor_x
        goal.target_pose.pose.position.y=self.anchor_y

        x,y,z,w = tf.transformations.quaternion_from_euler(self.anchor_x,self.anchor_y,self.anchor_theta)

        goal.target_pose.pose.orientation.x=x
        goal.target_pose.pose.orientation.y=y
        goal.target_pose.pose.orientation.z=z
        goal.target_pose.pose.orientation.w=w

        print   "going to anchor position to charge"
        self.ac.send_goal(goal)
        finished_within_time = self.ac.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.ac.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return  False
        else:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            return  True

    def callback(self,req):
        print req
        if req.charge   ==  1 :
            if self.nav2anchor() == False:
                return  SetChargeResponse("nav to anchor failed",False)
            time.sleep(1)
            if self.adjust() == False:
                return  SetChargeResponse("adjust failed",False)
            if self.backoff() == False:
                return  SetChargeResponse("adjust failed",False)
        if req.charge   ==  2 :
            if self.nav2anchor() == False:
                return  SetChargeResponse("nav to anchor failed",False)
        if req.charge   ==  3 :
            print "adjust()"
            if self.adjust() == False:
                print "adjust is err"
                return  SetChargeResponse("adjust failed",False)
            print "backoff()"
            if self.backoff() == False:
                print "backoff is err"
                return  SetChargeResponse("adjust failed",False)

            return  SetChargeResponse("charge success",True)
                


    def __init__(self):
        '''auto_charging'''
        rospy.init_node('auto_charging', anonymous=True)

        self.AutoChargingServer = rospy.Service('goto_charge', SetCharge, self.callback)

        self.anchor_x   =   rospy.get_param('anchor_x',self.anchor_x)
        self.anchor_y   =   rospy.get_param('anchor_y',self.anchor_y)
        self.anchor_theta   =   rospy.get_param('anchor_theta',self.anchor_theta)

        self.ac=actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
        self.ac.wait_for_server(rospy.Duration(60))
        rospy.spin()


       
        
if __name__ == '__main__':
    try:
        Charge()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")