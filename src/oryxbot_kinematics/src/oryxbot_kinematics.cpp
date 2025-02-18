#include "oryxbot_kinematics.h"

#define KINEMATICS_DEBUG false
#define INVERS_KINEMATICS_DEBUG false
#include "cmath"

template<typename T>
std::ostream& operator<< (std::ostream& os, std::vector<T> vec)
{
    os << '[';
    for(int i=0; i<vec.size(); i++) {
        os << vec[i] <<" ";
   }
    return os << ']';
}

std::vector<double> OryxBotKinematics::inverse_kinematics(double vx, double vy, double vth)
{
    std::vector<double> motor_speed(m_kinematics_mode, 0);
    switch(m_kinematics_mode) {
    case 2:
        motor_speed[0] = -(vx - vth*m_wheel_separation/2)/m_wheel_radius*30/PI;

        break;
    case 3:
        motor_speed[0] = (vx*sin(PI/3) + vy*cos(PI/3) + m_wheel_separation*vth)/m_wheel_radius*30/PI;
        motor_speed[1] = -(vx*sin(PI/3) - vy*cos(PI/3) - m_wheel_separation*vth)/m_wheel_radius*30/PI;
        motor_speed[2] = (-vy + m_wheel_separation*vth)/m_wheel_radius*30/PI;
        break;
     case 4:
        //ROS_INFO("4 wheel omni-directional, inverse_kinematics");
        motor_speed[0] = (vx + vy + 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;
        motor_speed[1] = -(vx - vy - 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;
        motor_speed[2] = -(vx + vy - 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;
        motor_speed[3] = (vx - vy + 0.5*vth*(m_width + m_length))/m_wheel_radius*30/PI;        
        break;
    default:
        ROS_INFO("cannot support the kinematics");
    }
#if INVERS_KINEMATICS_DEBUG
    std::cout << "kinematics motor speed: " << motor_speed << std::endl;
#endif
    return motor_speed;
}

//motorspeed (r/min)
std::vector<double> OryxBotKinematics::kinematics(std::vector<double> motorspeed)
{
    std::vector<double> velocity(3, 0);	// vx, vy, vth
    switch(m_kinematics_mode) {
    case 2:
        velocity[0] = (-motorspeed[0] + motorspeed[1])*PI*m_wheel_radius/60.0; //m/s
        velocity[1] = 0;
        velocity[2] = (motorspeed[0] + motorspeed[1])*2*PI*m_wheel_radius/m_wheel_separation/60.0; //rad/s
        break;
    case 3:
        velocity[0] = (motorspeed[0] - motorspeed[1])*PI/30.f*m_wheel_radius/sqrt(3);
        velocity[1] = (motorspeed[0] + motorspeed[1] - 2*motorspeed[2])*PI*m_wheel_radius/30.f
                        /3.f;
        velocity[2] = (motorspeed[0] + motorspeed[1] + motorspeed[2])*PI*m_wheel_radius/30.f
                        /3.f/m_wheel_separation;
        break;
     case 4:
        //ROS_INFO("4 wheels omni-directional, kinematics");
        velocity[0] = (motorspeed[0] - motorspeed[1])*m_wheel_radius/2*PI/30.f;
        velocity[1] = (motorspeed[0] - motorspeed[3])*m_wheel_radius/2*PI/30.f;
        velocity[2] = (motorspeed[1] + motorspeed[3])*m_wheel_radius/(m_width+m_length)*PI/30.f;

        break;
     default:
        ROS_INFO("error unknown kinematics_mode");
    }
#if KINEMATICS_DEBUG
    std::cout << "kinematics: "<< velocity << std::endl;
#endif
    return  velocity;
}


void OryxBotKinematics::vel_callback(const geometry_msgs::Twist::ConstPtr& vel)
{
    double vx = 0, vy = 0, vth = 0;
    if (fabs(vel->linear.x) > m_max_vx) {
        vx = fabs(vel->linear.x)/vel->linear.x*m_max_vx;
    } else {
        vx = vel->linear.x;
    }

    if (fabs(vel->linear.y) > m_max_vy) {
        vy = fabs(vel->linear.y)/vel->linear.y*m_max_vy;
    } else {
        vy = vel->linear.y;
    }

    if (fabs(vel->angular.z) > m_max_vth) {
        vth = fabs(vel->angular.z)/vel->angular.z*m_max_vth;
    } else {
        vth = vel->angular.z;
    }
#if INVERS_KINEMATICS_DEBUG
    std::cout <<"received velocity [vx vy vth] "
              << '[' << vx << " " << vy <<" " << vth << ']'
              << std::endl;
#endif
    m_motor_speed = inverse_kinematics(vx, vy, vth);
}

void OryxBotKinematics::status_callback(const oryxbot_msgs::car_data::ConstPtr& status)
{
    std::vector<double> motorspeed= status->speed;
    m_real_vel = kinematics(motorspeed);
}

void OryxBotKinematics::motor_speed_pub()
{
    m_vel_sub = m_h.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &OryxBotKinematics::vel_callback, this);
    m_motor_speed_pub = m_h.advertise<oryxbot_msgs::car_cmd>("/car_cmd", 10);

    ros::Rate loop(10);
    m_motor_speed = std::vector<double>(m_kinematics_mode, 0);
    while(ros::ok()) {
        loop.sleep();
        oryxbot_msgs::car_cmd msg;
        msg.speed = m_motor_speed;
        m_motor_speed_pub.publish(msg);
        //when call vel_callback, motorspeed will be overwrite;
        //clear old speed, when vel_callback is not called
        //m_motor_speed = std::vector<double>(m_kinematics_mode, 0);
        ros::spinOnce();
    }
}

//get bobac sensor information and kinematics
void OryxBotKinematics::status_pub()
{
    m_status_sub = m_h.subscribe<oryxbot_msgs::car_data>
                   ("car_data", 10, &OryxBotKinematics::status_callback, this);
    m_vel_pub = m_h.advertise<geometry_msgs::Twist>("real_vel", 10);
    m_real_vel = std::vector<double>(3, 0);

    ros::Rate loop(10);
    while(ros::ok()) {
#if KINEMATICS_DEBUG
        std::cout << "kinematics vel: " << m_real_vel << std::endl;
#endif
        geometry_msgs::Twist msg;
        msg.linear.x = m_real_vel[0];
        msg.linear.y = m_real_vel[1];
        msg.angular.z = m_real_vel[2];
        m_vel_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }
}


OryxBotKinematics::OryxBotKinematics()
{
    m_ph = ros::NodeHandle("~");
    //get kinematics_mode
    if(!m_ph.getParam("kinematics_mode", m_kinematics_mode)) {
        m_kinematics_mode = default_kinematics_mode;
        ROS_INFO("use default param \"kinematics_mode = %d\"",m_kinematics_mode);
    } else {
        ROS_INFO("kinematics_mode = %d", m_kinematics_mode);
    }

    //just for bobac, get wheel_diameter and wheel_separation
    if(!m_ph.getParam("wheel_radius", m_wheel_radius)) {
        m_wheel_radius = default_wheel_radius;
        ROS_INFO("use default param \"wheel_radius = %g\"",default_wheel_radius);
    } else {
        ROS_INFO("wheel_radius = %g", m_wheel_radius);
    }

    if(!m_ph.getParam("wheel_separation", m_wheel_separation)) {
        m_wheel_separation = default_wheel_separation;
        ROS_INFO("use default param \"wheel_separation = %g\"",default_wheel_separation);
    } else {
        ROS_INFO("wheel_separation = %g", m_wheel_separation);
    }

    //just for oryxbot, get width and length
    if(!m_ph.getParam("width", m_width)) {
        m_width = default_width;
        ROS_INFO("use default param \"width = %g\"",m_width);
    } else {
        ROS_INFO("width = %g", m_width);
    }
    
    if(!m_ph.getParam("length", m_length)) {
        m_length = default_length;
        ROS_INFO("use default param \"length = %g\"",m_length);
    } else {
        ROS_INFO("length = %g", m_length);
    }
    
    
    //get max velocity
    if(!m_ph.getParam("max_vx", m_max_vx)) {
        m_max_vx = default_vx;
        ROS_INFO("use default param \"max_vx = %g\"",m_max_vx);
    } else {
        ROS_INFO("max_vx = %g", m_max_vx);
    }

    if(!m_ph.getParam("max_vy", m_max_vy)) {
        m_max_vy = default_vy;
        ROS_INFO("use default param \"max_vy = %g\"",m_max_vy);
    } else {
        ROS_INFO("max_vy = %g", m_max_vy);
    }

    if(!m_ph.getParam("max_vth", m_max_vth)) {
        m_max_vth = default_vth;
        ROS_INFO("use default param \"max_vth = %g\"",m_max_vth);
    } else {
        ROS_INFO("max_vth = %g", m_max_vth);
    }
    m_ikthread = boost::thread(boost::bind(&OryxBotKinematics::motor_speed_pub, this));
    m_kthread = boost::thread(boost::bind(&OryxBotKinematics::status_pub, this));
}

