#include "oryxbot_arm_ar.h"
#include "ros/ros.h"

using namespace std;

#define CAMREA_X_MAX 0
#define CAMREA_Y_MAX 205
#define CAMREA_Z_MAX 170

ORYXBOT_ARM_AR::ORYXBOT_ARM_AR()
{
    artf_server = n.advertiseService("tf_ar",&ORYXBOT_ARM_AR::artf_callback,this);

    client_goto=n.serviceClient<oryxbot_msgs::pick_place>("goto_position");
    client_zero=n.serviceClient<oryxbot_msgs::set_zero>("set_zero");

    sub = n.subscribe("/oryxbot_arm_controller/position_info",100,&ORYXBOT_ARM_AR::position_callback,this);
    current_time = ros::Time::now();
    ROS_INFO("'tf_ar' node initialization completed ");
}

//pos_info回调函数
//发布机械臂末端到原点的TF
void ORYXBOT_ARM_AR::position_callback(const oryxbot_msgs::dobot_control& msg)
{
    //info当前位置
	position[0] = msg.x;
	position[1] = msg.y;
	position[2] = msg.z;
    position[3] = msg.r;

    //发布tf  robot-end
    geometry_msgs::Quaternion robot_quat = tf::createQuaternionMsgFromYaw(atan2(position[1],position[0]));
    current_time = ros::Time::now();
    geometry_msgs::TransformStamped robot_trans;
    robot_trans.header.stamp = current_time;
    robot_trans.header.frame_id = "robot";
    robot_trans.child_frame_id = "end";

    robot_trans.transform.translation.x = position[0]/1000;
    robot_trans.transform.translation.y = position[1]/1000;
    robot_trans.transform.translation.z = position[2]/1000;
    robot_trans.transform.rotation = robot_quat;
    robot_broadcaster.sendTransform(robot_trans);
    //ROS_INFO("X: %f  Y: %f Z: %f",position[0]/1000,position[1]/1000,position[2]/1000);
}

//三键标定
double position_calibration[4][3] = {0};
bool ORYXBOT_ARM_AR::artf_callback(oryxbot_msgs::pick_marker::Request &req,
                                 oryxbot_msgs::pick_marker::Response &res)
{
    if(req.mode==0){
        // //第一步归零      //归零时间不确定，再见，自己手动归零吧
        // {
        //     oryxbot_msgs::set_zero zero;
        //     zero.request.x=150;
        //     zero.request.y=0;
        //     zero.request.z=0;
        //     client_zero.call(zero);
        //     sleep(10);//延时10s
        // }
        //第二步先到达看ar码的最高点
        {
            oryxbot_msgs::goto_position pos;
            //走到摄像头固定位置-盒子的上方
            pos.request.x=CAMREA_X_MAX;
            pos.request.y=CAMREA_Y_MAX;
            pos.request.z=CAMREA_Z_MAX;
            client_goto.call(pos);
            sleep(10);//延时5s
        }
        //第三步获取字符串“ar_marker_number”
        {
            int a=req.number;
            stringstream ss;
            string stra="";
            ss<<"/ar_marker_";
            ss<<a;
            ss>>stra;
            const string &strb=stra;

            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("/robot",strb, ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform("/robot",strb, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex) 
            {
                ROS_ERROR("TF may have some problems.\n %s",ex.what());
                ros::Duration(1.0).sleep();
                res.success=false;
                res.message=" The AR was not found ! ";
                return true;
            }
            position_calibration[0][0]  =   transform.getOrigin().x()*1000.0;
            position_calibration[0][1]  =   transform.getOrigin().y()*1000.0;
            position_calibration[0][2]  =   transform.getOrigin().z()*1000.0;
            sleep(2);//延时5s
            ROS_INFO("%f %f %f ",position_calibration[0][0],position_calibration[0][1],position_calibration[0][2]);
        }
    }else if(req.mode==1){
        //第一步保存示教点
        {
            position_calibration[1][0]=position[0];
            position_calibration[1][1]=position[1];
            position_calibration[1][2]=position[2];
            ROS_INFO("%f %f %f ",position_calibration[1][0],position_calibration[1][1],position_calibration[1][2]);
        }
        //第二步提升高度
        {
            oryxbot_msgs::goto_position pos;
            //走到摄像头固定位置-盒子的上方
            pos.request.x=CAMREA_X_MAX;
            pos.request.y=CAMREA_Y_MAX;
            pos.request.z=CAMREA_Z_MAX;
            client_goto.call(pos);
            sleep(2);//延时5s
        }
        //第三步回零
        {
            oryxbot_msgs::set_zero zero;
            zero.request.x=150;
            zero.request.y=0;
            zero.request.z=0;
            client_zero.call(zero);
            sleep(6);//延时10s
        }
        //第四步走到ar码上方
        {
            oryxbot_msgs::goto_position pos;
            pos.request.x=position_calibration[1][0];
            pos.request.y=position_calibration[1][1];
            pos.request.z=position_calibration[1][2]+50;
            client_goto.call(pos);
            sleep(2);//延时5s
        }
        //第五步走到ar码上
        {
            oryxbot_msgs::goto_position pos;
            pos.request.x=position_calibration[1][0];
            pos.request.y=position_calibration[1][1];
            pos.request.z=position_calibration[1][2]+20;
            client_goto.call(pos);
            sleep(2);//延时5s            
        }
        req.position[0]=position_calibration[1][0];
        req.position[1]=position_calibration[1][1];
        req.position[2]=position_calibration[1][2];
    }else if(req.mode==2){
        position_calibration[2][0]=position[0];
        position_calibration[2][1]=position[1];
        position_calibration[2][2]=position[2];
        ROS_INFO("%f %f %f ",position_calibration[2][0],position_calibration[2][1],position_calibration[2][2]);

        position_calibration[3][0]=(position_calibration[2][1]-position_calibration[0][1])/1000.0;
        position_calibration[3][2]=(position_calibration[2][2]-position_calibration[0][2])/1000.0;
        ROS_INFO(" Please copy the line below  \n args=\"%f 0.0 %f  1.57079632679 0 3.1415926 end usb_cam 10\" />",position_calibration[3][0],position_calibration[3][2]);
    }

    res.success=true;
    return true;	
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"robot_ar");
    ORYXBOT_ARM_AR oryxbot_arm_ar;
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
