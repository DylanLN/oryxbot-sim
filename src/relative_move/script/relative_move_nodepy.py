#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''auto_charging ROS Node'''

import sys
import os

import rospy
import threading
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

from oryxbot_msgs.srv import SetRelativeMove
from oryxbot_msgs.srv import *

import tf

class PID_Controller(object):
    m_last_err = 0.0
    m_i_err = 0.0
    m_d_err = 0.0
    m_p=0.0
    m_i=0.0
    m_d=0.0

    def PID(self,err):
        self.m_i_err += self.m_last_err
        self.m_d_err = err - self.m_last_err
        k = self.m_p*err + self.m_i*self.m_i_err + self.m_d*self.m_d_err
        self.m_last_err = err
        if abs(k) < 0.01 :
            k=abs(k)/k*0.01
        if abs(k) > 0.08 :
            k=abs(k)/k*0.08
        return k

    def __init__(self):
        self.m_p=4
        self.m_p=1
        self.m_p=0
        self.m_last_err = 0
        self.m_i_err = 0
        self.m_d_err = 0


class RelMove(object):

    m_pub=''
    m_pose2d=Pose2D()

    def thread_job(self):
        rospy.spin()

    def pose2d_callback(self,pose):
        self.m_pose2d = pose
    
    def relative_move(self,req):
        if req.mode < 0 or req.mode >2 :
            return SetRelativeMoveResponse(False,"mode [0,2]")
        

        start_pose = self.m_pose2d
        end_pose = start_pose

        if req.mode==0:
            end_pose.x = start_pose.x + req.relative_move * math.cos(start_pose.theta)
            end_pose.y = start_pose.y + req.relative_move * math.sin(start_pose.theta)
        elif req.mode==1:
            end_pose.x = start_pose.x + req.relative_move * math.sin(start_pose.theta)
            end_pose.y = start_pose.y + req.relative_move * math.cos(start_pose.theta)
        elif req.mode==2:
            end_pose.theta = start_pose.theta + req.relative_move
        err = req.relative_move
        pid=PID_Controller()
        vel=Twist()
        r = rospy.Rate(50)
        while abs(err) > abs(m_err):
            if req.mode==0:
                vel.linear.x = pid.PID(err)
            elif req.mode==1:
                vel.linear.y = pid.PID(err)
            elif req.mode==2:
                vel.linear.z = pid.PID(err)
            self.m_pub.publish(vel)
            
            ii=0.0
            if req.relative_move<0:
                ii=-1.0
            else:
                ii=1.0
            if req.mode==0:
                err = ii*math.sqrt((end_pose.x - m_pose2d.x)*(end_pose.x - m_pose2d.x)+(end_pose.y - m_pose2d.y)*(end_pose.y - m_pose2d.y))
            elif req.mode==1:
		        err = ii*math.sqrt((end_pose.x - m_pose2d.x)*(end_pose.x - m_pose2d.x)+(end_pose.y - m_pose2d.y)*(end_pose.y - m_pose2d.y))
            elif req.mode==2:
                err = end_pose.theta - m_pose2d.theta
            r.sleep()

        vel.linear.x = 0.0    
        vel.linear.y = 0.0    
        vel.angular.z = 0.0
        self.m_pub.publish(vel)
        return SetRelativeMoveResponse(True,"current pose2d")
        

    def __init__(self):
        rospy.init_node('relative_move_node', anonymous=True)
        rospy.Subscriber('odom_pose2d', Pose2D, self.pose2d_callback)
        add_thread = threading.Thread(target = self.thread_job)
        add_thread.start()

        self.pose2d_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)
        s = rospy.Service('relative_move',SetRelativeMove,self.relative_move)
        rospy.spin()

def main():
    relmove=RelMove()

if __name__ == '__main__':
    main()


