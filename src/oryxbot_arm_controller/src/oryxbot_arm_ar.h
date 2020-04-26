#ifndef _ORYXBOT_ARM_AR_H
#define _ORYXBOT_ARM_AR_H

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "oryxbot_msgs/dobot_control.h"
#include "oryxbot_msgs/pick_place.h"
#include "oryxbot_msgs/goto_position.h"
#include "oryxbot_msgs/pick_marker.h"
#include "oryxbot_msgs/set_zero.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>       /* tan */
#include <iostream>
#include <unistd.h>

class ORYXBOT_ARM_AR
{
public:
    ORYXBOT_ARM_AR();

    ros::NodeHandle n;
	ros::ServiceServer artf_server;
    ros::ServiceClient client_goto,client_zero;
	ros::Subscriber sub;		
    tf::TransformListener listener;
    tf::TransformBroadcaster robot_broadcaster;
    ros::Time current_time;

    float position[4];

    void position_callback(const oryxbot_msgs::dobot_control& msg);
    bool artf_callback(oryxbot_msgs::pick_marker::Request &req,
                                        oryxbot_msgs::pick_marker::Response &res);

private:
    /* data */
};

#endif