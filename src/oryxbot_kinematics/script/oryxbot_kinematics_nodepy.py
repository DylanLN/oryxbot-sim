#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''OryxBotKinematics ROS Node'''
#导入rospy
import rospy
#导入sys、os和数学
import sys
import os
import math
#float32、twist、car——data、car——cmd
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from oryxbot_msgs.msg import car_data
from oryxbot_msgs.msg import car_cmd
#π
PI=3.1415926
#类OryxBotKinematics
class OryxBotKinematics(object):
#定义一些变量，主要是几轮、轮子半径、最大速度、车体长宽等
    m_kinematics_mode=4

    m_wheel_radius=0.25
    m_max_vx=0.5
    m_max_vy=0.5
    m_max_vth=1.0
    m_wheel_separation=0.5

    m_width=0.265
    m_length=0.245

    #oryx一般都是四麦克纳姆轮，其结构不会改其他轮式
    #运动学逆解  vx、vy、vth-->w0、w1、w2、w3
    def inverse_kinematics(self,vx,vy,vth):
        if self.m_kinematics_mode==2 :
            motor_speed = [0.0,0.0]
            return  motor_speed
        elif self.m_kinematics_mode==3 :
            motor_speed = [0.0,0.0,0.0]
            return motor_speed
        #只讲解mode=4为麦克纳姆轮时
        elif self.m_kinematics_mode==4 :
            #定义数组motor_speed
            motor_speed = [0.0,0.0,0.0,0.0]
            motor_speed[0] = (vx + vy + 0.5*vth*(self.m_width + self.m_length))/self.m_wheel_radius*30/PI
            motor_speed[1] = -(vx - vy - 0.5*vth*(self.m_width + self.m_length))/self.m_wheel_radius*30/PI
            motor_speed[2] = -(vx + vy - 0.5*vth*(self.m_width + self.m_length))/self.m_wheel_radius*30/PI
            motor_speed[3] = (vx - vy + 0.5*vth*(self.m_width + self.m_length))/self.m_wheel_radius*30/PI
            return motor_speed
    #运动学正解w0、w1、w2、w3-->vx、vy、vth
    def kinematics(self,motor_speed):
        velocity = [0.0,0.0,0.0]
        if self.m_kinematics_mode==2 :
            return velocity
        elif self.m_kinematics_mode==3 :
            return velocity
        #同样只讲解mode=4为麦克纳姆轮时
        elif self.m_kinematics_mode==4 :
            velocity[0] = (motor_speed[0] - motor_speed[1])*self.m_wheel_radius/2*PI/30.0
            velocity[1] = (motor_speed[0] - motor_speed[3])*self.m_wheel_radius/2*PI/30.0
            velocity[2] = (motor_speed[1] + motor_speed[3])*self.m_wheel_radius/(self.m_width+self.m_length)*PI/30.0
            return velocity
    #cmd_vel回调函数
    def vel_callback(self,vel):
        vx=0.0
        vy=0.0
        vth=0.0
        #对v进行限位
        if abs(vel.linear.x) > self.m_max_vx:
            vx = abs(vel.linear.x)/vel.linear.x*m_max_vx
        else:
            vx = vel.linear.x
        if abs(vel.linear.y) > self.m_max_vy:
            vy = abs(vel.linear.y)/vel.linear.y*m_max_vy
        else:
            vy = vel.linear.y
        if abs(vel.angular.z) > self.m_max_vth:
            vth = abs(vel.angular.z)/vel.angular.z*m_max_vth
        else:
            vth = vel.angular.z
        #对vx vy vth进行运动学逆解
        m_motor_speed = self.inverse_kinematics(vx,vy,vth)
        msg=car_cmd()
        #将逆解所得轮子速度赋给msg.speed
        msg.speed = m_motor_speed
        #发布信息到主题car_cmd
        self.m_motor_speed_pub.publish(msg)

    #car_data回调函数
    def cardata_callback(self,status):
        motor_speed = status.speed
        #对于接收的speed进行运动学正解
        m_real_vel = self.kinematics(motor_speed)
        #创建Twist格式的消息
        msg = Twist()
        #msg赋值
        msg.linear.x = m_real_vel[0]
        msg.linear.y = m_real_vel[1]
        msg.angular.z = m_real_vel[2]
        #发布信息到主题real_vel
        self.m_vel_pub.publish(msg)
    #类构造函数
    def __init__(self):
        #初始化呢node
        rospy.init_node('oryxbot_kinematics', anonymous=True)
        #订阅car_data话题，回调函数为cardata_callback()
        rospy.Subscriber('car_data', car_data, self.cardata_callback)
        #订阅cmd_vel话题，回调函数为vel_callback
        rospy.Subscriber('cmd_vel', Twist, self.vel_callback)
        #发布car_cmd话题
        self.m_motor_speed_pub = rospy.Publisher('car_cmd', car_cmd, queue_size=10)
        #发布real_vel话题
        self.m_vel_pub = rospy.Publisher('real_vel', Twist, queue_size=10)
        #开始订阅
        rospy.spin()

#main()函数
def main():
    #new一个OryxBotKinematics的对象
    oryxbotkinematics = OryxBotKinematics()
#ifmain
if __name__ == '__main__':
    main()

