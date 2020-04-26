#ifndef _ORYXBOT_PICK_AR_H
#define _ORYXBOT_PICK_AR_H

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "oryxbot_msgs/dobot_control.h"
#include "oryxbot_msgs/pick_place.h"
#include "oryxbot_msgs/goto_position.h"
#include "oryxbot_msgs/pick_marker.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>       /* tan */
#include <iostream>
#include <unistd.h>

class ORYXBOT_PICK
{
public:
    ORYXBOT_PICK();

    ros::NodeHandle n;
	ros::ServiceServer pick_server,place_server;
    ros::ServiceClient client_pick,client_goto,client_place;
	ros::Subscriber sub;		
    tf::TransformListener listener;
    tf::TransformBroadcaster robot_broadcaster;
    ros::Time current_time, last_time;

    float position[4];

    void position_callback(const oryxbot_msgs::dobot_control& msg);
    bool pick_callback( oryxbot_msgs::pick_marker::Request &req,
                        oryxbot_msgs::pick_marker::Response &res);
    bool place_callback( oryxbot_msgs::pick_place::Request &req,
                        oryxbot_msgs::pick_place::Response &res);

private:
    /* data */
};

#endif
