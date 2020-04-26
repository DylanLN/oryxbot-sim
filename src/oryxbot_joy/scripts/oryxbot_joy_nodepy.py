#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''oryxbot_joy_pnode ROS Node'''
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopJoy(object):

    pub=''
    i_velLinear_x=1
    i_velLinear_y=0
    i_velAngular=3
    f_velLinearMax=0.2
    f_velAngularMAx=2.0

    def callback(self,data):
        twist   =   Twist()
        twist.linear.x  =   data.axes[1]
        twist.linear.y  =   data.axes[0]
        twist.angular.z =   data.axes[3]
        self.pub.publish(twist)

    def __init__(self):
        '''oryxbot_joy_pnode Publisher'''
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('oryxbot_joy_pnode', anonymous=True)
        self.i_velLinear_x   =   rospy.get_param('axis_linear_x',self.i_velLinear_x)
        self.i_velLinear_y   =   rospy.get_param('axis_linear_y',self.i_velLinear_y)
        self.i_velAngular   =   rospy.get_param('axis_angular',self.i_velAngular)
        self.f_velLinearMax =   rospy.get_param('linear_max',self.f_velLinearMax)
        self.f_velAngularMAx    =   rospy.get_param('angular_max',self.f_velAngularMAx)
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.spin()
        
        
if __name__ == '__main__':
    joy=TeleopJoy()
