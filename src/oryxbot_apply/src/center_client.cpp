#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "oryxbot_msgs/centerAction.h"
#include <actionlib/client/terminal_state.h>
#include <std_srvs/SetBool.h>
 
using namespace  std;

typedef actionlib::SimpleActionClient<oryxbot_msgs::centerAction> center_client;

class CENTERCLIENT
{
public:
    CENTERCLIENT();
    ros::NodeHandle nh;

    // void doneCb(const actionlib::SimpleClientGoalState& state, 
    //             const robot_msgs::centerResultConstPtr& result);
    // void activeCb();
    // void feedbackCb(const robot_msgs::centerFeedbackConstPtr& feedback);
    ros::ServiceServer server;
    center_client client;
    bool client_callback(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res);
private:
    /* data */
};

CENTERCLIENT::CENTERCLIENT():client("center_server",true)
{
    //等待服务器
    client.waitForServer();
    server = nh.advertiseService("center_client",&CENTERCLIENT::client_callback,this);
    ROS_INFO("init ok!");
}

/*
 *action完成时的回调函数，一次性
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const oryxbot_msgs::centerResultConstPtr& result)
{
    ROS_INFO("DONE");
    //ros::shutdown();
}

/*
 *action启动时的回调函数，一次性
 */
void activeCb()
{
    ROS_INFO("ACTIVE");
}

/*
 *action收到反馈时的回调函数
 */
void feedbackCb(const oryxbot_msgs::centerFeedbackConstPtr& feedback)
{
    ROS_INFO("THE NUMBER RIGHT NOM IS: %s",feedback->message.c_str());
}

bool CENTERCLIENT::client_callback(std_srvs::SetBool::Request &req,
                                   std_srvs::SetBool::Response &res)
{
    //创建一个action的goal
    oryxbot_msgs::centerGoal goal;

    if(req.data>=0)
    {
        goal.point_number=req.data;
        //发送action的goal到服务器，并设置回调函数。
        client.sendGoal(goal,doneCb,activeCb,feedbackCb);

    }else{
        client.cancelAllGoals();
    }
    return true;
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "center_client");
    CENTERCLIENT centerclient;

    ros::AsyncSpinner spinner(2); 
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

