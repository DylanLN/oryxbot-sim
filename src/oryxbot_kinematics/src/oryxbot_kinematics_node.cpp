#include "oryxbot_kinematics.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oryxbot_kinematics");
    OryxBotKinematics ok;
    ros::spin();
    return 0;
}
