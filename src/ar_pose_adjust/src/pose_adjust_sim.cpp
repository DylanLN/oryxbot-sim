#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>
#include "oryxbot_msgs/SetArPose.h"
#include "oryxbot_msgs/ar_pose.h"
#include <iostream>
#include <tf/transform_listener.h>

#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"

//宏定义参数
#define X_GOAL (0.2)
#define Y_GOAL (0)
#define Z_GOAL (0)
#define THETA_GOAL (0)
// velocity control
#define P_THETA (0.7)
#define P_Y (0.6)
#define P_X (0.9)

#define I_THETA (0)
#define I_Y (0)
#define I_X (0)

#define D_THETA (0.06)
#define D_Y (0.2)
#define D_X (0.1)

#define ADJUST_TIMES (1)
#define REPEAT_TIMES (2)
//pid计算类
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

class PoseAdj
{
public:
    PoseAdj();
    ~PoseAdj();
    int flag, tmp;

    void tf_echo(void);
    bool adjust(oryxbot_msgs::SetArPose::Request &req, oryxbot_msgs::SetArPose::Response &res);
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceServer service;
    ros::Publisher pub;
    oryxbot_msgs::ar_pose curr_pose;
    tf::TransformListener listener;
    boost::thread tf_thread;
};
//类PoseAdj构造函数
PoseAdj::PoseAdj()
{

    //提供服务/pose_adjust/adjust_service
    service = n.advertiseService("/pose_adjust/adjust_service", &PoseAdj::adjust, this);
    //发布话题cmd_vel
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    curr_pose.rpy.resize(3);
    curr_pose.rpy[0] = curr_pose.rpy[1] = curr_pose.rpy[2] = 0;
    tf_thread = boost::thread(boost::bind(&PoseAdj::tf_echo, this));
}
//析构函数
PoseAdj::~PoseAdj()
{}

void PoseAdj::tf_echo(void)
{
    ros::Rate loop(10);
    while(ros::ok()){
        //监听tf
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/usb_cam_link","/ar_marker_0", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/usb_cam_link","/ar_marker_0", ros::Time(0), transform);
            curr_pose.position.x =transform.getOrigin().x();
            curr_pose.position.y=transform.getOrigin().y();
            curr_pose.position.z=transform.getOrigin().z();
            transform.getBasis().getRPY(curr_pose.rpy[0], curr_pose.rpy[1], curr_pose.rpy[2]);
            curr_pose.confidence=1;
        }
        catch (tf::TransformException &ex) 
        {
            curr_pose.position.x =0;
            curr_pose.position.y=0;
            curr_pose.position.z=0;
            curr_pose.confidence=0;
        }
        loop.sleep();
    }

}

//服务/pose_adjust/adjust_service回调函数
bool PoseAdj::adjust(oryxbot_msgs::SetArPose::Request &req, oryxbot_msgs::SetArPose::Response &res)
{
    ros::Rate loop(10);
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;

    //x方向的偏差
    double x_err  = curr_pose.position.x - X_GOAL;
    double z_goal[ADJUST_TIMES] = {0};
    for(int i=0; i<ADJUST_TIMES; i++) {
        z_goal[i] = curr_pose.position.z-(curr_pose.position.z - Z_GOAL)*(i+1)/ADJUST_TIMES;
        ROS_INFO_STREAM("z_goal[" << z_goal[i] << "]");
    }

    bool track_flag = curr_pose.confidence;

    if(req.adjust) {
        for(int k=0; k<REPEAT_TIMES; k++) {	//repeat
            for (int i=0; i<ADJUST_TIMES; i++) {
                ROS_INFO_STREAM("adjust times[" << i << "]");
                //th偏差
                double theta_err = curr_pose.rpy[1] - THETA_GOAL;
                //实例化类x_pid
                PID_Controller x_pid(P_X, I_X, D_X);
                //实例化类y_pid
                PID_Controller y_pid(P_Y, I_Y, D_Y);
                //实例化类theta_pid
                PID_Controller theta_pid(P_THETA, I_THETA, D_THETA);
                //更新x_err
                x_err = curr_pose.position.x - X_GOAL;
                //y_err
                double y_err = 0;
                y_err= curr_pose.position.y - Y_GOAL;
                int sum=0;
                //进行pid计算
                while(((fabs(theta_err) > 0.01)||(fabs(x_err) > 0.01)||(fabs(y_err) > 0.01))) {

                    if(curr_pose.confidence==0){
                        sum++;
                        if (sum>300)
                        {
                            res.message = "time out";
                            res.success = false;
                            return true;
                        }
                        twist.linear.x = 0;
                        twist.linear.y = 0;
                        twist.angular.z = 0;
                    }else{
                        sum=0;
                        twist.angular.z = -theta_pid.PID(theta_err);
                        twist.linear.x = x_pid.PID(x_err);
                        x_err  = curr_pose.position.x - X_GOAL;
                        twist.linear.y = y_pid.PID(y_err);
                        ROS_INFO_STREAM("x_err:" << x_err);
                        theta_err = curr_pose.rpy[1] ;

                        y_err  = curr_pose.position.y - Y_GOAL;
                        //x_err  = curr_pose.position.x ;
                        track_flag = curr_pose.confidence;
                        //发布消息。
                    }
                    pub.publish(twist);
                    //延时
                    loop.sleep();
                }
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                pub.publish(twist);
                usleep(500*1000);
            }// end repeat
            //如果ar码位置消息不可信，则返回失败
            // if(!track_flag) {
            //     twist.linear.x = 0;
            //     twist.linear.y = 0;
            //     twist.angular.z = 0;
            //     pub.publish(twist);
            //     res.message = "flag failed";
            //     res.success = false;
            //     return true;
            // }
        }
        res.message = "success";
        res.success = true;
        return true;
    } else {
        res.message = "failed";
        res.success = false;
        return true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_adjust");
    //实例化类
    PoseAdj PoseAdj;
    ros::spin();
    // //创建多线程接收（2）
    // ros::AsyncSpinner spinner(2);
    // //开始多线程接收
    // spinner.start();
    ros::waitForShutdown();
}
