#!/usr/bin/env python
#Filename:TestModule.py

#import sys

import rospy
from oryxbot_msgs.srv import *

# Called when a client sends a message
def open_camera(s):
	rospy.wait_for_service('oryxbot_interaction')
    # create a handle to the add_two_ints service
	Standard_mode = rospy.ServiceProxy('oryxbot_interaction', standard_mode)
    # formal style
	resp2 = Standard_mode.call(standard_modeRequest(0, 1))
	print "open_camera"
	print s
	if resp2.success==1:
		return True
	else:
		return False


def Hello(s):
    print ("Hello World")
    print(s)
