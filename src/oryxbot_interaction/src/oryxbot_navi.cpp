#include <ros/ros.h>
#include <oryxbot_msgs/nav_goal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* navclient;


class ORYXBOT_NAVGOAL
{
public:
  ORYXBOT_NAVGOAL();
  ros::NodeHandle n;

  move_base_msgs::MoveBaseGoal goal;  //目标点
  ros::Subscriber posesub;		
	ros::ServiceServer navi_server;
  bool navi_deal(oryxbot_msgs::nav_goal::Request &req,
					    oryxbot_msgs::nav_goal::Response &res);

private:
  /* data */
};

ORYXBOT_NAVGOAL::ORYXBOT_NAVGOAL()
{
  navi_server=n.advertiseService("oryxbot_navgoali",&ORYXBOT_NAVGOAL::navi_deal,this);
	ROS_INFO("init ok");
}

bool ORYXBOT_NAVGOAL::navi_deal(oryxbot_msgs::nav_goal::Request &req,
					    oryxbot_msgs::nav_goal::Response &res)
{
      
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = req.pose.x;
    goal.target_pose.pose.position.y = req.pose.y;

    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(req.pose.theta);
    goal.target_pose.pose.orientation.x = orientation.x;
    goal.target_pose.pose.orientation.y = orientation.y;
    goal.target_pose.pose.orientation.z = orientation.z;
    goal.target_pose.pose.orientation.w = orientation.w;

	ROS_INFO("Sending goal");
    navclient->sendGoal(goal);
  	navclient->waitForResult();
    if(navclient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("move");
      res.success=true;
    }
    else
    {
      ROS_INFO("failed");
      res.success=false;
    }
    return true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
    navclient = new MoveBaseClient ("move_base", true);
	while(!navclient->waitForServer(ros::Duration(5.0))){
  		ROS_INFO("Waiting for the move_base action server to come up");
	}
    ORYXBOT_NAVGOAL oryxbot_navgoal;
    ros::spin();
    return 0;
}
