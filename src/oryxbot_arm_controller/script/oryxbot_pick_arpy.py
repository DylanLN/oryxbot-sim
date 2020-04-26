#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''robot_pick ROS Node'''
# license removed for brevity

import sys
import os
import math
import threading

import rospy
import time
from oryxbot_msgs.srv import *
from oryxbot_msgs.msg import *


import tf

class ORYXBOT_PICK(object):
    CAMREA_X_MAX=20.0
    CAMREA_Y_MAX=205.0
    CAMREA_Z_MAX=170.0

    position=[0.0,0.0,0.0,0.0]
    pick_server=''
    place_server=''
    client_pick=''
    client_place=''
    client_goto=''
    listener=''

    def thread_job(self):
        rospy.spin()

    def callback(self,msg):
        self.position[0] = msg.x/1000.0
        self.position[1] = msg.y/1000.0
        self.position[2] = msg.z/1000.0
        self.position[3] = msg.r
        br = tf.TransformBroadcaster()
        br.sendTransform((self.position[0], self.position[1], self.position[2]),
                        tf.transformations.quaternion_from_euler(0, 0, math.atan2(self.position[1],self.position[0])),
                        rospy.Time.now(),
                        "end",
                        "robot")
        

    def pick_callback(self,req):
        xx=self.position[0]
        yy=self.position[1]
        zz=self.position[2]

        a=req.number
        marker="/ar_marker_"+str(req.number)
        print marker

        if req.mode==0:
            goresp = self.client_goto(self.CAMREA_X_MAX,self.CAMREA_Y_MAX,self.CAMREA_Z_MAX,0.0)
            if goresp.ret==False:
                return pick_markerResponse([self.CAMREA_X_MAX,self.CAMREA_Y_MAX,self.CAMREA_Z_MAX],False,"  goto error  !")
            time.sleep(1)
        elif req.mode==1:
            pass
        else:
            return pick_markerResponse([0,0,0],False,"  Pattern error  !")

        try:
            (trans,rot) = self.listener.lookupTransform('/robot', marker, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return pick_markerResponse([0,0,0],False,"  tf erro !")
        x_=trans[0]*1000.0
        y_=trans[1]*1000.0
        z_=trans[2]*1000.0
        print trans
        print x_,y_,z_
        goresp = self.client_goto(x_,y_,z_+50,0.0)
        if goresp.ret==False:
            return pick_markerResponse([x_,y_,z_+50],False,"  goto error  !")

        piresp = self.client_pick(x_,y_,z_,0.0)
        if piresp.ret==False:
            return pick_markerResponse([x_,y_,z_],False,"  Manipulator unattainable !")

        goresp = self.client_goto(x_,y_,z_+50,0.0)
        if goresp.ret==False:
            return pick_markerResponse([x_,y_,z_+50],False,"  goto error  !")
        time.sleep(1)

        if abs(req.position[0])<0.01 and abs(req.position[1])<0.01 and abs(req.position[2])<0.01 :
            if req.mode == 0:
                goresp = self.client_goto(self.CAMREA_X_MAX,self.CAMREA_Y_MAX,self.CAMREA_Z_MAX,0.0)
                if goresp.ret==False:
                    return pick_markerResponse([self.CAMREA_X_MAX,self.CAMREA_Y_MAX,self.CAMREA_Z_MAX],False,"  goto error  !")
            elif req.mode == 1:
                goresp = self.client_goto(xx,yy,zz,0.0)
                if goresp.ret==False:
                    return pick_markerResponse([xx,yy,zz],False,"  goto error  !")
        else:
            goresp = self.client_goto(req.position[0],req.position[1],req.position[2]+50,0.0)
            if goresp.ret==False:
                return pick_markerResponse([req.position[0],req.position[1],req.position[2]+50],False,"  goto error  !")
            plresp = self.client_place(req.position[0],req.position[1],req.position[2],0.0)
            if plresp.ret==False:
                return pick_markerResponse([req.position[0],req.position[1],req.position[2]],False,"  place error  !")
        return pick_markerResponse([0,0,0],True," ok !")

    def place_callback(self,req):
        print req
        goresp = self.client_goto(req.x,req.y,req.z+40,0.0)
        if goresp.ret==False:
            return pick_placeResponse(False)
        plresp = self.client_place(req.x,req.y,req.z,0.0)
        if plresp.ret==False:
            return pick_placeResponse(False)
        return pick_placeResponse(True)
        

    def __init__(self):
        '''robot_pick'''
        rospy.init_node('robot_pick', anonymous=True)
        
        self.listener = tf.TransformListener()
        self.pick_server = rospy.Service('pick_ar', pick_marker, self.pick_callback)
        self.place_server = rospy.Service('place_ar', pick_place, self.place_callback)
        rospy.Subscriber('/oryxbot_arm_controller/position_info', dobot_control, self.callback)
        rospy.wait_for_service('pick')
        rospy.wait_for_service('place')
        rospy.wait_for_service('goto_position')

        self.client_pick=rospy.ServiceProxy("pick",pick_place)
        self.client_place=rospy.ServiceProxy("place",pick_place)
        self.client_goto=rospy.ServiceProxy("goto_position",pick_place)

        add_thread = threading.Thread(target = self.thread_job)
        add_thread.start()
        
        rospy.spin()

     
if __name__ == '__main__':
    try:
        ORYXBOT_PICK()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
