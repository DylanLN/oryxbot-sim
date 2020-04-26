/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>	   
 */

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include "unistd.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#include "oryxbot_msgs/pick_place.h"
#include "oryxbot_msgs/dobot_control.h"
#include "oryxbot_msgs/goto_position.h"
using namespace std;

std::string read_data;
serial::Serial _serial;				// serial object
oryxbot_msgs::dobot_control pos;

//获取位姿
bool get_pos()
{
	std::string Gcode = "",	data="";
	Gcode = (std::string)"P2220" + "\r\n";
	_serial.write(Gcode.c_str());

	data = _serial.read(_serial.available());
	//std::cout << data << endl;

    std::vector<std::string> v;
    std::string::size_type pos1, pos2, pos3;
    pos1 = data.find("X");
    pos2 = data.find("Y");
    pos3 = data.find("Z");

	v.push_back(data.substr(pos1+1,pos2-pos1));	//X
	v.push_back(data.substr(pos2+1,pos3-pos2));	//Y
	v.push_back(data.substr(pos3+1,data.length()-pos3-1));	//Z

	pos.x	=	std::atof(v[0].c_str());
	pos.y	=	std::atof(v[1].c_str());
	pos.z	=	std::atof(v[2].c_str());
}

//goto pos
bool goto_pos(float gx,float gy,float gz)
{
	std::string Gcode = "";
	std_msgs::String result;
	ros::Rate loop_rate(20);
	int i=0;
	float record_y=1.0;
	record_y = pos.y;
	if(record_y/gy<0){
		char x[10];
		char y[10];
		char z[10];


		sprintf(x, "%.2f", 200.0);
		sprintf(y, "%.2f", 0.0);
		sprintf(z, "%.2f", 100.0);
		Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
		ROS_INFO("%s", Gcode.c_str());
		_serial.write(Gcode.c_str());
		result.data = _serial.read(_serial.available());

		while (!(abs(pos.x-200.0)<1&&abs(pos.y-0.0)<1&&abs(pos.z-100.0)<1))
		{
			/* code for loop body */
			get_pos();
			loop_rate.sleep();
			i++;
			if(i>400)
				return false;
		}
	}



	char x[10];
	char y[10];
	char z[10];
	sprintf(x, "%.2f", gx);
	sprintf(y, "%.2f", gy);
	sprintf(z, "%.2f", gz);
	Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());

	while (!(abs(pos.x-gx)<1&&abs(pos.y-gy)<1&&abs(pos.z-gz)<1))
	{
		/* code for loop body */
		get_pos();
		loop_rate.sleep();
		i++;
		if(i>400)
			return false;
	}
	return true;
	
}

//goto pos
bool goto_position_deal(oryxbot_msgs::goto_position::Request &req,
						oryxbot_msgs::goto_position::Response &res)
{
	sleep(1);
	res.ret=goto_pos(req.x,req.y,req.z);
	return true;
}
//放置
bool place_deal(oryxbot_msgs::pick_place::Request &req, 
				oryxbot_msgs::pick_place::Response &res)
{
	sleep(1);
	res.ret=goto_pos(req.x,req.y,req.z);
	if(!res.ret)
		return true;

	std::string Gcode = "";
	Gcode = (std::string)"M2231 V0" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	read_data = _serial.read(_serial.available());
	return true;
}


//抓取
bool pick_deal(oryxbot_msgs::pick_place::Request &req, 
				oryxbot_msgs::pick_place::Response &res)
{
	sleep(1);
	res.ret=goto_pos(req.x,req.y,req.z);
	if(!res.ret){
		res.ret=false;
		return true;
	}

	std::string Gcode = "";
	Gcode = (std::string)"M2231 V1" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	int i = _serial.write(Gcode.c_str());
	res.ret=1;
	read_data = _serial.read(_serial.available());
	return true;
}

/* 
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "swiftpro_write_node");
	ros::NodeHandle nh;

	ros::ServiceServer pick_server = nh.advertiseService("pick",pick_deal);
	ros::ServiceServer place_server = nh.advertiseService("place",place_deal);
	ros::ServiceServer goto_position_server = nh.advertiseService("goto_position",goto_position_deal);

	ros::Publisher 	 pub = nh.advertise<oryxbot_msgs::dobot_control>("oryxbot_arm_controller/position_info", 1);
	ros::Rate loop_rate(10);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ros::Duration(3.5).sleep();				// wait 3.5s
		_serial.write("M2120 V0\r\n");			// stop report position
		ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M17\r\n");				// attach
		ros::Duration(0.1).sleep();				// wait 0.1s
		ROS_INFO_STREAM("Attach and wait for commands");
	}

	while (ros::ok())
	{
		get_pos();
		if(abs(pos.x)<0.1&&abs(pos.y)<0.1&&abs(pos.z)<0.1)
		{
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		pub.publish(pos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}


