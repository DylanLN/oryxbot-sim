#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''odometry_publisher ROS Node'''
#导入rospy
import rospy
#导入sys os
import sys
import os
#导入math-数学 和  多线程所需要的库
import math
import threading

#导入时间和tf
import time
import tf

#导入消息nav_msgs/Odometry、geometry_msgs/Twist、geometry_msgs/Pose2D、geometry_msgs/TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TransformStamped

#类Odometry_odom
class Odometry_odom(object):
#定义一些变量
    pose2d_pub=''
    odom_pub=''
    x=0.0
    y=0.0
    th=0.0
    vx=0.0
    vy=0.0
    vth=0.0

    last_time=''
    current_time=''

    def vel_callback(self,data):
        self.vx=data.linear.x
        self.vy=data.linear.y
        self.vth=data.angular.z
        
        self.current_time=rospy.get_rostime()
        dt = (self.current_time.to_sec() -self.last_time.to_sec())
        delta_x = (self.vx*math.cos(self.th)-self.vy*math.sin(self.th))*dt
        delta_y = (self.vx*math.sin(self.th)-self.vy*math.cos(self.th))*dt
        delta_th = self.vth*dt
        self.x +=delta_x
        self.y +=delta_y
        self.th +=delta_th
        self.last_time  =   self.current_time

#多线程工作函数，一直rospy.spin()进行查询消息
    def thread_job(self):
        rospy.spin()
#类初始化函数
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)
        rospy.Subscriber('real_vel', Twist, self.vel_callback)
        self.pose2d_pub = rospy.Publisher('odom_pose2d', Pose2D, queue_size=50)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        
        self.last_time=rospy.get_rostime()
        self.last_time=rospy.get_rostime()

        add_thread = threading.Thread(target = self.thread_job)
        add_thread.start()
        
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            pose2d=Pose2D()
            pose2d.x=self.x
            pose2d.y=self.y
            pose2d.theta=self.th
            self.pose2d_pub.publish(pose2d)

            br = tf.TransformBroadcaster()
            br.sendTransform((self.x, self.y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.th),
                            rospy.Time.now(),
                            "base_footprint",
                            "odom")
            
            odom=Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"

            #set the position
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0,0,self.th)

            #set the velocity
            odom.child_frame_id = "base_footprint"
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.vth
            #publish the message
            self.odom_pub.publish(odom)
            r.sleep()


if __name__ == '__main__':
    odom1=Odometry_odom()

