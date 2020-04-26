#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import tf
import threading
import PyKDL
import rospkg
import time

from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from oryxbot_msgs.msg import dobot_control
from oryxbot_msgs.srv import pick_place
from oryxbot_msgs.srv import goto_position
from oryxbot_msgs.srv import pick_marker

GRIPPER_OPEN = [0.0,0.0]
GRIPPER_CLOSED = [0.0,0.0]

class RobotControl(object):
    joint_pose=[0.0,0.0]
    joint_cmd=[0.0,0.0]
    #vel=[0.0,0.0]
    
    def __init__(self):

        self.pick_server = rospy.Service('pick', pick_place, self.pick_callback)
        self.place_server = rospy.Service('place', pick_place, self.place_callback)
        self.joint_pub = rospy.Publisher('/magician_eff_group_controller/command', JointTrajectory, queue_size=10)
        self.state_sub = rospy.Subscriber('/magician_eff_group_controller/state',JointTrajectoryControllerState,self.state_callback,queue_size=10)


        self.goto_server = rospy.Service('goto_position', goto_position, self.goto_callback)
        self.pose_pub = rospy.Publisher('/oryxbot_arm_controller/position_info', dobot_control, queue_size=10)


        # 是否需要使用笛卡尔空间的运动规划
        #self.cartesian = rospy.get_param('~cartesian', False)
        moveit_commander.roscpp_initialize(sys.argv)

        self.group=moveit_commander.MoveGroupCommander('magician_arm')

        self.eff=MoveGroupCommander("magician_eff_group")
        # 控制夹爪张开
        self.eff.set_joint_value_target(GRIPPER_OPEN)
        self.eff.go()

        # 获取终端link的名称
        self.end_effector_link = self.group.get_end_effector_link()
        #print self.end_effector_link
        #
        self.group.set_pose_reference_frame("magician_base")

        # 当运动规划失败后，允许重新规划
        self.group.allow_replanning(True)

        #允许误差
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.01)


        self.group.set_planning_time(9)
        #self.group.set_planner_id("RRTConnectkConfigDefault")

        self.group.set_named_target('home')
        self.group.go()

        #self.target_pose = PoseStamped()
        add_thread = threading.Thread(target = self.thread_job)
        add_thread.start()
        add_thread = threading.Thread(target = self.thread_joint)
        add_thread.start()
        rospy.spin()

    def state_callback(self,msg):
        for i in range(len(msg.joint_names)):
            self.joint_pose[i] = msg.actual.positions[i]

    def pick_callback(self,req):
        target_pose = PoseStamped()
        target_pose = self.group.get_current_pose(self.end_effector_link)
        print target_pose
        target_pose.header.frame_id = "base_footprint"
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = req.x/1000.0
        target_pose.pose.position.y = req.y/1000.0
        target_pose.pose.position.z = req.z/1000.0
        target_pose.pose.orientation.x =0.0
        target_pose.pose.orientation.y =0.0
        target_pose.pose.orientation.z =-0.70710678
        target_pose.pose.orientation.w =0.70710678

        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target_pose, self.end_effector_link)
        traj = self.group.plan()
        self.group.execute(traj)

        self.joint_cmd[0]=0.035
        self.joint_cmd[1]=-0.035

        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown() and self.joint_pose[0]-self.joint_pose[1] <0.0451:
        #     rate.sleep()
        time.sleep(4)
        
        return True

    def place_callback(self,req):
        target_pose = PoseStamped()
        target_pose = self.group.get_current_pose(self.end_effector_link)
        print target_pose
        target_pose.header.frame_id = "base_footprint"
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = req.x/1000.0
        target_pose.pose.position.y = req.y/1000.0
        target_pose.pose.position.z = req.z/1000.0
        target_pose.pose.orientation.x =0.0
        target_pose.pose.orientation.y =0.0
        target_pose.pose.orientation.z =-0.70710678
        target_pose.pose.orientation.w =0.70710678

        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target_pose, self.end_effector_link)
        traj = self.group.plan()
        self.group.execute(traj)

        self.joint_cmd[0]=0.0
        self.joint_cmd[1]=0.0

        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown() and self.joint_pose[0]-self.joint_pose[1] > 0.37:
        #     rate.sleep()
        time.sleep(3)
        return True


    def thread_job(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            gpose = self.group.get_current_pose(self.end_effector_link)
            ppose=dobot_control()
            ppose.x=gpose.pose.position.x*1000.0
            ppose.y=gpose.pose.position.y*1000.0
            ppose.z=gpose.pose.position.z*1000.0
            self.pose_pub.publish(ppose)
            rate.sleep()

    def thread_joint(self):
        rate = rospy.Rate(10) # 10hz
        max_vel=30
        while not rospy.is_shutdown():
            traj = JointTrajectory()
            traj.joint_names = ['finger_joint1','finger_joint2']
            point = JointTrajectoryPoint()
            pos = self.joint_pose[0]
            cmd = self.joint_cmd[0]
            point.positions.append(cmd)
            point.velocities.append(-(cmd-pos)*max_vel)
            #point.accelerations.append(5)
            point.time_from_start.secs=1
            point.time_from_start.nsecs=0            

            pos = self.joint_pose[1]
            cmd = self.joint_cmd[1]
            point.positions.append(cmd)
            point.velocities.append(-(cmd-pos)*max_vel)
            #point.accelerations.append(5)
            point.time_from_start.secs=1
            point.time_from_start.nsecs=0
            point.effort=[100,100]

            traj.points.append(point)

            self.joint_pub.publish(traj)
            rate.sleep()


    def goto_callback(self,req):
        target_pose = PoseStamped()
        target_pose = self.group.get_current_pose(self.end_effector_link)
        #print target_pose
        target_pose.header.frame_id = "base_footprint"
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = req.x/1000.0
        target_pose.pose.position.y = req.y/1000.0
        target_pose.pose.position.z = req.z/1000.0
        target_pose.pose.orientation.x =0.0
        target_pose.pose.orientation.y =0.0
        target_pose.pose.orientation.z =-0.70710678
        target_pose.pose.orientation.w =0.70710678

        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target_pose, self.end_effector_link)
        traj = self.group.plan()
        self.group.execute(traj)
        if traj.joint_trajectory.header.frame_id=="/base_footprint" :
            return True
        else:
            return False





if __name__=='__main__':  
    rospy.init_node('robot_control')
    robot_control=RobotControl()  
