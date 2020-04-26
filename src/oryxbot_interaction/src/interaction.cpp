#include "ros/ros.h"
#include "oryxbot_msgs/standard_mode.h"

#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

uint8_t usb_state=0,ar_state=0;

//进程id返回函数。
pid_t getProcessPidByName(const char *proc_name)
{
     FILE *fp;
     char buf[100];
     char cmd[200] = {'\0'};
     pid_t pid = -1;
     sprintf(cmd, "pidof %s", proc_name);
     if((fp = popen(cmd, "r")) != NULL)
     {
         if(fgets(buf, 255, fp) != NULL)
         {
             pid = atoi(buf);
         }
     }
     printf("pid = %d \n", pid);
     pclose(fp);
     return pid;
}


//相机回调函数
bool usbcam_callback(oryxbot_msgs::standard_mode::Request &req,
                                 oryxbot_msgs::standard_mode::Response &res)
{
    if(req.mode==1&&req.number==0&&usb_state==0){
        if(system("roslaunch /home/ln/oryxbot_ws/src/ar_pose_adjust/launch/usb_cam_with.launch&")<0){
            ROS_ERROR("system() errorr");
        }else{
            usb_state=1;
        }
    }else if(req.mode==0&&req.number==0&&usb_state==1){
        int killa;
        killa=getProcessPidByName("/opt/ros/kinetic/lib/usb_cam/usb_cam_node");
        kill(killa,SIGTERM);
        usb_state=0;
    }else if(req.mode==1&&req.number==1&&ar_state==0){
        if(system("roslaunch /home/ln/oryxbot_ws/src/ar_pose_adjust/launch/ar_track_camera.launch&")<0){
            ROS_ERROR("system() errorr");
        }else{
            ar_state=1;
        }
    }else if(req.mode==0&&req.number==1&&ar_state==1){
        int killa;
        killa=getProcessPidByName("/home/ln/oryxbot_ws/devel/lib/ar_track_alvar/individualMarkersNoKinect");
        kill(killa,SIGTERM);
        ar_state=0;
    }
    return true;	
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"oryxbot_interaction");
    ros::NodeHandle n;

	ros::ServiceServer intera=n.advertiseService("oryxbot_interaction",usbcam_callback);
    ros::spin();
    return 0;
}



