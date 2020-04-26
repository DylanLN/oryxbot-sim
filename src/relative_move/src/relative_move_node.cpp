#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose2D.h"
#include "oryxbot_msgs/SetRelativeMove.h"
#include "boost/thread.hpp"

class PID_Controller
{
public:
    PID_Controller(double p, double i, double d):m_p(p), m_i(i), m_d(d)
    {
        m_last_err = 0;
        m_i_err = 0;
        m_d_err = 0;
    }
    ~PID_Controller() {}
    double PID(double err)
    {
        m_i_err += m_last_err;
        m_d_err = err - m_last_err;
        double k = m_p*err + m_i*m_i_err + m_d*m_d_err;
        m_last_err = err;
        if(fabs(k) < 0.01) k = fabs(k) / k * 0.01;
        if(fabs(k) > 0.08 ) k = fabs(k) / k * 0.08;
        return k;
    }
private:
    double m_p;
    double m_i;
    double m_d;
    double m_last_err;
    double m_i_err;
    double m_d_err;
};


class RelMove
{
public:
    RelMove():m_nh(ros::NodeHandle()),
        m_nh_private(ros::NodeHandle("~")),
        use_amcl(false)
    {
        m_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        sub_odom = m_nh.subscribe("odom_pose2d", 10, &RelMove::pose2d_callback, this);

//        m_nh_private.param<bool>("use_amcl", use_amcl, false);
        m_nh_private.param<double>("p", m_p, 100);
//        m_nh_private.param<double>("i", m_i, 0);
//        m_nh_private.param<double>("d", m_d, 100);

        m_nh_private.param<double>("err", m_err, 0.05);

        srv_relmove = m_nh.advertiseService("relative_move", &RelMove::relative_move, this);


    }

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_amcl_pose;
    ros::ServiceServer srv_relmove;
    ros::Publisher m_pub;
    bool use_amcl;
    double m_p;
//    double m_i;
//    double m_d;
    double m_err;



    geometry_msgs::Pose2D m_pose2d;
//    geometry_msgs::PoseWithCovarianceStamped m_amcl_pose;

    void pose2d_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        m_pose2d = *msg;
        //ROS_INFO_STREAM("get odom msg");
    }
    bool relative_move(oryxbot_msgs::SetRelativeMove::Request &req,
                       oryxbot_msgs::SetRelativeMove::Response &res)
    {
        //get param
	if (req.mode<0 || req.mode >2){
            res.message = "mode [0,2]";
            res.success = false;
            return true;
        }
	
        geometry_msgs::Pose2D start_pose = m_pose2d;
        geometry_msgs::Pose2D end_pose = start_pose;
	switch (req.mode) {
	case 0:
		end_pose.x = start_pose.x + req.relative_move * cos(start_pose.theta);
		end_pose.y = start_pose.y + req.relative_move * sin(start_pose.theta);
        	break;
	case 1:
		end_pose.x = start_pose.x + req.relative_move * sin(start_pose.theta);
		end_pose.y = start_pose.y + req.relative_move * cos(start_pose.theta);
		break;
	case 2: 
		end_pose.theta = start_pose.theta + req.relative_move;
		break;	
	}

        ROS_INFO_STREAM("end x[" << end_pose.x << "]" << "end y ["<< end_pose.y << "]");

        double err = (req.relative_move);

        geometry_msgs::Twist vel;
        PID_Controller pid(m_p, 0, 0);
        while( fabs(err) > fabs(m_err) ){

	    switch (req.mode) {
	    case 0:
		vel.linear.x = pid.PID(err);
		break;
	    case 1:
		vel.linear.y = pid.PID(err);
		break;
	    case 2:
		vel.angular.z = pid.PID(err);
		break;
	    }	
            
            float ii=0.0; 
            m_pub.publish(vel);
            usleep(10*1000);
	    switch (req.mode) {
	    case 0:
       		if(req.relative_move<0) ii=-1.0;else ii=1.0;
		err = ii*sqrt((end_pose.x - m_pose2d.x)*(end_pose.x - m_pose2d.x)+(end_pose.y - m_pose2d.y)*(end_pose.y - m_pose2d.y));
		break;
	    case 1:
        	if(req.relative_move<0) ii=-1.0;else ii=1.0;
		err = ii*sqrt((end_pose.x - m_pose2d.x)*(end_pose.x - m_pose2d.x)+(end_pose.y - m_pose2d.y)*(end_pose.y - m_pose2d.y));
		break;
	    case 2:
		err = end_pose.theta - m_pose2d.theta;
		break;
	    }

           // ROS_INFO_STREAM("err[" << err << "]");
            ros::spinOnce();

        }
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.angular.z = 0;
        m_pub.publish(vel);
        char str[255]= {0};
        sprintf(str, "current pose2d[%3f, %3f, %3f]", m_pose2d.x, m_pose2d.y, m_pose2d.theta);
        //set message
        res.message = std::string(str);
        res.success = true;
        return true;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "relative_move_node");

    RelMove rm;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
