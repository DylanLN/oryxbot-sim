#ifndef _BOBAC_KINEMATICS_H
#define _BOBAC_KINEMATICS_H

#include "string"
#include "vector"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"
#include "oryxbot_msgs/car_data.h"
#include "oryxbot_msgs/car_cmd.h"

const double PI = std::acos(-1);

#define default_kinematics_mode (2)

#define default_wheel_radius (0.25)
#define default_wheel_separation (0.5)
#define default_vx (0.5)
#define default_vy (0.5)
#define default_vth (1.0)

#define default_width (0.265)
#define default_length (0.245)

class OryxBotKinematics
{
public:
    OryxBotKinematics();
    ros::NodeHandle m_h;
    ros::NodeHandle m_ph;//private NodeHandle, for parameter set
    ros::Subscriber m_vel_sub; //subscribe topic = "cmd_vel"
    ros::Subscriber m_status_sub; //
    ros::Publisher m_motor_speed_pub, m_vel_pub;
    std::vector<double> m_motor_speed;	//inverse_kinematics
    std::vector<double> m_real_vel;	//kinematics

    double m_wheel_radius;
    double m_max_vx;
    double m_max_vy;
    double m_max_vth;

    int m_kinematics_mode;
    //just for 2/3 wheels
    double m_wheel_separation;    

    //just for four wheels omni-directional
    double m_width;
    double m_length;
    
    std::vector<double> kinematics(std::vector<double> motorspeed);
    std::vector<double> inverse_kinematics(double vx, double vy, double vth);

    void vel_callback(const geometry_msgs::Twist::ConstPtr& vel);
    void status_callback(const oryxbot_msgs::car_data::ConstPtr& status);
    void motor_speed_pub();
    void status_pub();

    boost::thread m_ikthread;
    boost::thread m_kthread;
};

#endif
