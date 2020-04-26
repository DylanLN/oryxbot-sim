#include <ros/ros.h>
#include <oryxbot_msgs/nav_goal.h>
#include <tf/transform_broadcaster.h>

using namespace std;


int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_goals");
    ros::NodeHandle n;
    ros::ServiceClient client_nav=n.serviceClient<oryxbot_msgs::nav_goal>("oryxbot_navgoali");


    oryxbot_msgs::nav_goal goal;

    goal.request.pose.position.x=-1.0;
    goal.request.pose.position.y=-0.5;
    goal.request.pose.position.z=0.0;

    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0);

    goal.request.pose.orientation.x = orientation.x;
    goal.request.pose.orientation.y = orientation.y;
    goal.request.pose.orientation.z = orientation.z;
    goal.request.pose.orientation.w = orientation.w;

    while (ros::ok())
    {
        /* code for loop body */
        if(client_nav.call(goal))
        {
		ROS_INFO("X: %f , y: %f ",goal.request.pose.position.x,goal.request.pose.position.y);
            if(goal.request.pose.position.y<1.5)
                goal.request.pose.position.y=goal.request.pose.position.x+0.1;
            else if(!(goal.request.pose.position.y<1.5)&&goal.request.pose.position.x<0.5)
            {
                geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(1.5707963);
                goal.request.pose.orientation.x = orientation.x;
                goal.request.pose.orientation.y = orientation.y;
                goal.request.pose.orientation.z = orientation.z;
                goal.request.pose.orientation.w = orientation.w;

                goal.request.pose.position.x=goal.request.pose.position.x+0.1;

            }else if(goal.request.pose.position.y>=1.5&&goal.request.pose.position.y>=1.5){
                break;
                return 0;
            }

        }
        

    }
    return 0;
}
