#include "ros/ros.h"
#include "oryxbot_msgs/goto_position.h"
#include "oryxbot_msgs/dobot_control.h"

class ORYXBOT_SINGLE 
{
public:
    ORYXBOT_SINGLE();
    float position[4];
    ros::NodeHandle n;
    ros::Subscriber sub;		
    ros::ServiceClient client_goto;
	ros::ServiceServer single_server;
    bool control_deal(oryxbot_msgs::goto_position::Request &req,
					    oryxbot_msgs::goto_position::Response &res);
    void position_callback(const oryxbot_msgs::dobot_control& msg);

private:
    /* data */

};

//pos_info回调函数
void ORYXBOT_SINGLE::position_callback(const oryxbot_msgs::dobot_control& msg)
{
    //info当前位置
	position[0] = msg.x;
	position[1] = msg.y;
	position[2] = msg.z;
    position[3] = msg.r;
}

bool ORYXBOT_SINGLE::control_deal(oryxbot_msgs::goto_position::Request &req,
					    oryxbot_msgs::goto_position::Response &res)
{
    oryxbot_msgs::goto_position pos;
    pos.request.x=position[0]+req.x;
    pos.request.y=position[1]+req.y;
    pos.request.z=position[2]+req.z;
    client_goto.call(pos);
}

ORYXBOT_SINGLE::ORYXBOT_SINGLE()
{
    client_goto=n.serviceClient<oryxbot_msgs::goto_position>("goto_position");
	single_server=n.advertiseService("oryxbot_arm_single",&ORYXBOT_SINGLE::control_deal,this);
    sub = n.subscribe("/oryxbot_arm_controller/position_info",100,&ORYXBOT_SINGLE::position_callback,this);
}

// JSBool testjs(JSContext *cx, uint32_t argc, jsval *vp)
// {
//     if (argc > 0) {
//         JSString *string = NULL;
//         JS_ConvertArguments(cx, argc, JS_ARGV(cx, vp), "S", &string);
//         if (string) {
//             JSStringWrapper wrapper(string);
//             //这块就有值了
//             CCLog(wrapper.get().c_str());
//         }
//     }
//     CCLog("js function");
//     return JS_TRUE;
    
// }

int main(int argc,char** argv)
{
    ros::init(argc,argv,"oryxbot_arm_single");
    ORYXBOT_SINGLE oryxbot_single;

    ros::spin();
    return 0;
}