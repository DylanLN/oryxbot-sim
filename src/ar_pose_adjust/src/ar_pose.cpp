#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <oryxbot_msgs/ar_pose.h>
#include <oryxbot_msgs/SetCharge.h>

int track=127;
oryxbot_msgs::ar_pose pose;
ros::Publisher pub;
//ar_pose_pub::orientation rpy
//ar_pose_single话题回调函数
void pose_callback(const ar_track_alvar_msgs::AlvarMarker marker_msgs)
{
    pose.rpy.resize(3);	
    //ROS_INFO_STREAM("marker_msgs.confidence:" << marker_msgs.confidence);

    if (marker_msgs.id==track||track==127)
    {
        /* code for True */
        //消息置信度为1，将id和pose等复制
        if(marker_msgs.confidence == 1) {
            pose.name = marker_msgs.id;
            pose.position.x = marker_msgs.pose.pose.position.x;
            pose.position.y = marker_msgs.pose.pose.position.y;
            pose.position.z = marker_msgs.pose.pose.position.z;
            pose.confidence = marker_msgs.confidence ;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(marker_msgs.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(pose.rpy[0], pose.rpy[1], pose.rpy[2]);
            pub.publish(pose);
        } else {
            //否则全部设为0
            pose.name = 0;
            pose.position.x = 0;
            pose.position.y = 0;
            pose.position.z = 0;
            pose.rpy[0] = pose.rpy[1] = pose.rpy[2] = 0;
            pose.confidence = 0;
        }
        //发布话题ar_pose
    }
}

bool TrackCallback(oryxbot_msgs::SetCharge::Request &req, 
                   oryxbot_msgs::SetCharge::Response &res)
{
    track=req.charge;
    res.success=true;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_pose");
    ros::NodeHandle n;
    //订阅ar_pose_single话题
    ros::Subscriber sub  = n.subscribe("ar_pose_single", 10, pose_callback);
    //发布ar_pose话题
    pub = n.advertise<oryxbot_msgs::ar_pose>("ar_pose", 10);
    ros::ServiceServer Track_Server = n.advertiseService("Track", TrackCallback);
    ros::spin();
}
