#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <pthread.h>

#include "DobotDll.h"
#include "ros/ros.h"
#include "oryxbot_msgs/dobot_control.h"
#include "oryxbot_msgs/set_zero.h"
#include "oryxbot_msgs/get_zero.h"
#include "oryxbot_msgs/pick_place.h"
#include "oryxbot_msgs/pick_marker.h"
#include "oryxbot_msgs/standard_mode.h"


#include "oryxbot_msgs/goto_position.h"
#include <sensor_msgs/JointState.h>

#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/bind.hpp"

using namespace std;

class Dobot_controller
{
public:
	Dobot_controller();
	~Dobot_controller();
	string  port;	


private:
	ros::NodeHandle m_handle;
	ros::Publisher position_pub; 	
	ros::Publisher joint_pub;
	ros::Subscriber position_sub;
	ros::ServiceServer set_zero_server;
	ros::ServiceServer get_zero_server;
	ros::ServiceServer pick_server;
	ros::ServiceServer place_server;
	ros::ServiceServer goto_position_server;
	ros::ServiceServer teach_server;	//示教
	ros::ServiceServer sucker_server;//吸盘控制

	
	boost::thread pub_thread;
	oryxbot_msgs::dobot_control pub_position;
	sensor_msgs::JointState  arm_joint;

	int robot_init();

	bool set_endeffector_params(float x,float y,float z);
	bool set_ptp_params();
	bool get_robot_pose(Pose* pose);
	bool set_robot_pose(vector<float> position);
	bool set_suctioncup(bool state);
	
	void position_callback(const oryxbot_msgs::dobot_control msg);
	void pub_func();	
	bool set_zero_deal(oryxbot_msgs::set_zero::Request &req, oryxbot_msgs::set_zero::Response &res);
	bool get_zero_deal(oryxbot_msgs::get_zero::Request &req, oryxbot_msgs::get_zero::Response &res);
	bool pick_deal(oryxbot_msgs::pick_place::Request &req, oryxbot_msgs::pick_place::Response &res);
	bool place_deal(oryxbot_msgs::pick_place::Request &req, oryxbot_msgs::pick_place::Response &res);
	bool goto_position_deal(oryxbot_msgs::goto_position::Request &req, oryxbot_msgs::goto_position::Response &res);
	bool teach_deal(oryxbot_msgs::standard_mode::Request &req,oryxbot_msgs::standard_mode::Response &res);
	bool sucker_deal(oryxbot_msgs::standard_mode::Request &req,oryxbot_msgs::standard_mode::Response &res);

};



