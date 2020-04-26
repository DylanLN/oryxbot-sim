#include "oryxbot_pick_ar.h"
#include "ros/ros.h"
#include <sstream>

using namespace std;

#define CAMREA_X_MAX 20
#define CAMREA_Y_MAX 205
#define CAMREA_Z_MAX 170

//pos_info回调函数
void ORYXBOT_PICK::position_callback(const oryxbot_msgs::dobot_control& msg)
{
    //info当前位置
	position[0] = msg.x;
	position[1] = msg.y;
	position[2] = msg.z;
    position[3] = msg.r;
    //ROS_INFO("pos X:  %.3f  Y:  %.3f Z:  %.3f",position[0],position[1],position[2]);

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
    //ROS_INFO("X: %.1f  Y: %.1f Z: %.1f",position[0],position[1],position[2]);
}

//pick_ar回调函数
bool ORYXBOT_PICK::pick_callback(oryxbot_msgs::pick_marker::Request &req,
                                 oryxbot_msgs::pick_marker::Response &res)
{
    //获取字符串“ar_marker_number”
    int a=req.number;
    stringstream ss;
    string stra="";
    ss<<"/ar_marker_";
    ss<<a;
    ss>>stra;
    const string &strb=stra;
    
    oryxbot_msgs::goto_position pos;
    if(req.mode==0)//mode=0，则机械臂跑到盒子上面抓取物体
    {
        //走到摄像头固定位置-盒子的上方
        pos.request.x=CAMREA_X_MAX;
        pos.request.y=CAMREA_Y_MAX;
        pos.request.z=CAMREA_Z_MAX;
        client_goto.call(pos);

        //计算当前与（0,190,170）的距离，放在这里也可以是因为position还未更新。
        float line=0.0;
        line=pow(position[0]-CAMREA_X_MAX,2)+pow(position[1]-CAMREA_Y_MAX,2)+pow(position[0]-CAMREA_Z_MAX,2);

        sleep(line/25000.0);//延时一段时间让摄像头休整一下
    }else if(req.mode==1){

    }else{
        res.message="  Pattern error  !";
        res.success=false;
        return true;
    }

    //监听tf
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
        return true;
    }

    float x_,y_,z_;
    x_=transform.getOrigin().x()*1000.0;
    y_=transform.getOrigin().y()*1000.0;
    z_=transform.getOrigin().z()*1000.0;
    ROS_INFO("r-a X:  %.3f  Y:  %.3f Z:  %.3f",x_,y_,z_);

    //走到ar码上方
    pos.request.x=x_;
    pos.request.y=y_;
    pos.request.z=z_+50;
    client_goto.call(pos);
    // pos.request.x=x_;
    // pos.request.y=y_;
    // pos.request.z=z_;
    // client_goto.call(pos);
    // 抓取
    oryxbot_msgs::pick_place pick;
    pick.request.x=x_;
    pick.request.y=y_;
    pick.request.z=z_;

    if(client_pick.call(pick))
    {
        if(pick.response.ret==true)
            res.success=true;
        else
        {
            res.message=" Manipulator unattainable !";
            res.success=false;
        }
    }
    else
    {
        res.success=false;
    }

    sleep(0.7);//延时一段时间不能频繁call/goto服务

    //回到ar码上方
    pos.request.x=x_;
    pos.request.y=y_;
    pos.request.z=z_+50;
    client_goto.call(pos);

    sleep(0.3);//延时一段时间不能频繁call/goto服务

    if(!((abs(req.position[0])<0.001)&&(abs(req.position[1])<0.001)&&(abs(req.position[2])<0.001))){
        //走到放置位置上方
        ROS_INFO("goto");	
        pos.request.x=req.position[0];
        pos.request.y=req.position[1];
        pos.request.z=req.position[2]+50;
        client_goto.call(pos);
        //ROS_INFO("gooto end");
        

        //下去放下
        oryxbot_msgs::pick_place place;
        place.request.x=req.position[0];
        place.request.y=req.position[1];
        place.request.z=req.position[2];
        client_place.call(place);
	    ROS_INFO("place end");
    }else{
        if(req.mode==0)
        {
            ROS_INFO("固定位置");

            //回到摄像头固定位置
            pos.request.x=CAMREA_X_MAX;
            pos.request.y=CAMREA_Y_MAX;
            pos.request.z=CAMREA_Z_MAX;
            client_goto.call(pos);
        }else if(req.mode==1){
            pos.request.x=position[0];
            pos.request.y=position[1];
            pos.request.z=position[2];
            client_goto.call(pos);
            ROS_INFO("mode == 1");
        }
    }

    return true;	
}

//place_ar回调函数
bool ORYXBOT_PICK::place_callback(oryxbot_msgs::pick_place::Request &req,
                                 oryxbot_msgs::pick_place::Response &res)
{
    //走到放置位置上方
    oryxbot_msgs::goto_position pos;
    pos.request.x=req.x;
    pos.request.y=req.y;
    pos.request.z=req.z+50;
    client_goto.call(pos);    
    //下去放下
    oryxbot_msgs::pick_place place;
    place.request.x=req.x;
    place.request.y=req.y;
    place.request.z=req.z;
    client_place.call(place);

	res.ret = true;
    return true;
}

ORYXBOT_PICK::ORYXBOT_PICK()
{
    pick_server=n.advertiseService("pick_ar",&ORYXBOT_PICK::pick_callback,this);
    place_server=n.advertiseService("place_ar",&ORYXBOT_PICK::place_callback,this);

    client_pick=n.serviceClient<oryxbot_msgs::pick_place>("pick");
    client_place=n.serviceClient<oryxbot_msgs::pick_place>("place");
    client_goto=n.serviceClient<oryxbot_msgs::pick_place>("goto_position");

    sub = n.subscribe("/oryxbot_arm_controller/position_info",100,&ORYXBOT_PICK::position_callback,this);
    current_time = ros::Time::now();
    ROS_INFO("IS OK");
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"robot_pick");
    ORYXBOT_PICK oryxbot_pick;
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
