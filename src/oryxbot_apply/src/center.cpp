#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdio.h>
#include <stddef.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "actionlib/server/simple_action_server.h"
#include "oryxbot_msgs/centerAction.h"
#include <oryxbot_msgs/nav_goal.h>
#include <oryxbot_msgs/pick_marker.h>
#include <oryxbot_msgs/goto_position.h>
#include <oryxbot_msgs/set_zero.h>
#include <oryxbot_msgs/SetCharge.h>
#include <oryxbot_msgs/SetRelativeMove.h>

/***************************************************************************
//中心调度节点

//输入包括一个调用调度的action和暂停的service

***************************************************************************/

typedef actionlib::SimpleActionServer<oryxbot_msgs::centerAction> center_server;

using namespace std;

//获取参数
template<typename T>
T getParam(const string& name,const T& defaultValue)
{
    T v;
    if(ros::param::get(name,v))
    {
        //ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        //cout << name << " : " << v <<endl;
        return v;
    }
    else 
        ROS_WARN_STREAM("Cannot find value for parameter: --"<<name<<"-- ,\tassigning default: "<<defaultValue);
    return defaultValue;
}

/**************************************************************
//以下是point数据定义部分
**************************************************************/
typedef struct {
    float pose[3];
}Pose;

typedef struct {
    int fun;
    int num;
    vector<float> parameter;
}Action;

typedef struct {
    string name;
    Pose pose;
    int setp;
    vector<Action> action;
}Point;

/**************************************************************
//以下是path数据定义部分
**************************************************************/
typedef struct {
    string prior;//上一个节点
    string next;//下一个节点
    string point_name;//当前节点编码
    int number;//当前节点编号
    Point point;
}Path;
vector<Path> path;//路径

//读取点位函数
//以下是point参数所需str
const string param_str="/center";
    //const string point_str="/point0"
        const string name_str="/name";        
        const string pose_str="/pose";
            const string xyth_str[3]={"/x","/y","/th"};
        const string setp_str="/setp";
        const string action_str="/action";
            const string fun_str="/fun";
            const string num_str="/num";
            const string parameter_str="/parameter";
bool readpoint(string point_name,Point *point1)
{
        stringstream ss;
        string str="";
        cout << "point :" << point_name << endl;

        //name
        for(int j=0;j<1;j++)
        {
            ss<<param_str<<point_name<<name_str;
            ss>>str;
            const string &str1=str;
            (*point1).name = getParam<string>(str1,"");
            str="";
            ss.str("");
            ss.clear();
        }

        //pose
        for(int j=0;j<3;j++)
        {
            ss<<param_str<<point_name<<pose_str<<xyth_str[j];
            ss>>str;
            const string &str0=str;
            (*point1).pose.pose[j]=getParam<float>(str0,0.0);
            str="";
            ss.str("");
            ss.clear();
        }

        //setp
        for(int j=0;j<1;j++)
        {
            ss<<param_str<<point_name<<setp_str;
            ss>>str;
            const string &str1=str;
            (*point1).setp=getParam<int>(str1,0);
            str="";
            ss.str("");
            ss.clear();
        }

        //action
        for(int j=0;j<(*point1).setp;j++)
        {
            Action action1;
            //fun
            for(int k=0;k<1;k++)
            {
                ss<<param_str<<point_name<<action_str<<j<<fun_str;
                ss>>str;
                const string &str1=str;
                action1.fun=getParam<int>(str1,0);
                str="";
                ss.str("");
                ss.clear();
            }

            //num
            for(int k=0;k<1;k++)
            {
                ss<<param_str<<point_name<<action_str<<j<<num_str;
                ss>>str;
                const string &str1=str;
                action1.num=getParam<int>(str1,0);
                str="";
                ss.str("");
                ss.clear();
            }

            //parameter
            for(int k=0;k<action1.num;k++)
            {
                ss<<param_str<<point_name<<action_str<<j<<parameter_str<<k;
                ss>>str;
                const string &str1=str;
                action1.parameter.push_back(getParam<float>(str1,0));
                str="";
                ss.str("");
                ss.clear();
            }
            (*point1).action.push_back(action1);
        }
    return true;
}

//读取路径
bool readpath(const string path_name)
{
    const string param_str="/center";
        //const string path_str="/path";
            string point_str="/point0";
                const string prior_str="/prior";
                const string next_str="/next";

    path.clear();//第一步清空当前路径

    stringstream ss;
    string str="";
    int i=0;
    while (ros::ok())
    {
        /* code for loop body */
        Path temp;

        //prior
        ss<<param_str<<path_name<<point_str<<prior_str;
        ss>>str;
        const string &str0=str;
        temp.prior = getParam<string>(str0,"end");
        str="";
        ss.str("");
        ss.clear();

        //next
        ss<<param_str<<path_name<<point_str<<next_str;
        ss>>str;
        const string &str1=str;
        temp.next = getParam<string>(str1,"end");
        str="";
        ss.str("");
        ss.clear();      

        //point
        if(temp.prior=="start")
            temp.point_name="/point0";
        else
            temp.point_name=path[i-1].next;

        //number
        temp.number=i;

        //point
        readpoint(temp.point_name,&(temp.point));

        path.push_back(temp);

        if(temp.next=="end")break;
        else if(temp.next=="ring")break;
        point_str=temp.next;
        i++;
    }
    cout << path_name <<" : ";
    for(int i=0;i<path.size();i++)
        cout << path[i].point_name << " --> ";
    cout << path[path.size()-1].next << endl;
    return true;
}


/**************************************************************
//类
**************************************************************/
class CENTRALCONTROLLING
{
public:
    CENTRALCONTROLLING();
    ros::NodeHandle nh;
    ros::ServiceClient client_nav,cilent_pick,client_charging,client_relativemove,client_gotopos,client_setzero,client_track;
    ros::ServiceServer server_suspend;
    center_server centerserver;
    uint8_t suspend;
    std::vector<string> route;
    

    void delay_s(float time);
    int action(int number, int fun,int num,vector<float> parameter);
    void execute(const oryxbot_msgs::centerGoalConstPtr &goal, center_server *as);
    bool suspend_callback(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res);
private:
    /*data*/
};

//构造函数
CENTRALCONTROLLING::CENTRALCONTROLLING():centerserver(nh,"center_server",boost::bind(&CENTRALCONTROLLING::execute,this,_1,&centerserver),false)
{
    centerserver.start();
    suspend=0;
    server_suspend=nh.advertiseService("center_suspend",&CENTRALCONTROLLING::suspend_callback,this);

    client_nav  =nh.serviceClient<oryxbot_msgs::nav_goal>("oryxbot_navgoali");//导航
    cilent_pick =nh.serviceClient<oryxbot_msgs::pick_marker>("pick_ar");//1.抓取放置	5
    client_charging =nh.serviceClient<oryxbot_msgs::SetCharge>("goto_charge");//2.自动回冲	1
    client_relativemove =nh.serviceClient<oryxbot_msgs::SetRelativeMove>("relative_move");//3.定向移动	2
    client_gotopos  =nh.serviceClient<oryxbot_msgs::goto_position>("goto_position");//4.手臂goto	4
    client_setzero  =nh.serviceClient<oryxbot_msgs::set_zero>("set_zero");//5.手臂归零	4
    client_track    =nh.serviceClient<oryxbot_msgs::SetRelativeMove>("track_ar");//6.ar码追踪 2
}

//秒级延时函数
void CENTRALCONTROLLING::delay_s(float time)
{
    usleep((int)(time*1000.0*1000.0));
}

//程序暂停回调函数
bool CENTRALCONTROLLING::suspend_callback(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res)
{
    suspend=req.data;
    return true;
}

//action执行函数
const string function_name[8] = {"navgition","pick_ar","auto_charging","relative_move","goto_position","track"};
int CENTRALCONTROLLING::action(int number,int fun,int num,vector<float> parameter)
{
    int value=0;
    
    if(fun<=5||fun>=0)    //fun 0~3
    {
        ROS_INFO(" fun : %s",function_name[fun].c_str());
    }
    else
    {
        ROS_INFO(" Please check the error here. It should be fun parameter error.");
        return false;
    }

    if(fun==0){             //0.延时
        //延时 parameter[0] s;
        delay_s(parameter[0]);
	    value=true;
    }else if (fun==1){      //1.pick
        /* code for True */
        oryxbot_msgs::pick_marker   srv;
        srv.request.number=parameter[0];
        srv.request.mode=parameter[1];
        srv.request.position[0]=parameter[2];
        srv.request.position[1]=parameter[3];
        srv.request.position[2]=parameter[4];
        if(cilent_pick.call(srv))
        {
            value=srv.response.success;
        }else{
            ROS_ERROR(" pick_ar service not found !");
            value=false;   
        }
    }else if (fun==2){      //2.自动回冲
        /* code for True */
        oryxbot_msgs::SetCharge srv;
        srv.request.charge=parameter[0];
        if(client_charging.call(srv))
        {
            value=srv.response.success;
        }else{
            ROS_ERROR(" charging service not found !");
            value=false;   
        }
    }else if (fun==3){      //3.定向移动
        /* code for True */
        oryxbot_msgs::SetRelativeMove srv;
        srv.request.relative_move=parameter[0];
        srv.request.mode=parameter[1];
        if(client_relativemove.call(srv))
        {
            value=srv.response.success;
        }else{
            ROS_ERROR(" relativemove service not found !");
            value=false;   
        }
    }else if(fun==4){       //4.手臂goto
        oryxbot_msgs::goto_position srv;
        srv.request.x=parameter[0];
        srv.request.y=parameter[1];
        srv.request.z=parameter[2];
        srv.request.r=parameter[3];
        if(client_gotopos.call(srv))
        {
            value=srv.response.ret;
        }else{
            ROS_ERROR(" gotopos service not found !");
            value=false;   
        }
    }else if (fun==5){      //5.手臂回零
        /* code for True */
        oryxbot_msgs::set_zero srv;
        srv.request.x=parameter[0];
        srv.request.y=parameter[1];
        srv.request.z=parameter[2];
        srv.request.r=parameter[3];
        if(client_setzero.call(srv))
        {
            value=srv.response.ret;
        }else{
            ROS_ERROR(" setzero service not found !");
            value=false;   
        }
    }else if (fun==6){      //6.ar码追踪
        /* code for True */
        delay_s(2);
        oryxbot_msgs::SetRelativeMove srv;
        srv.request.mode=parameter[0];
        srv.request.relative_move=parameter[1];
        if(client_track.call(srv))
        {
            value=srv.response.success;
        }else{
            ROS_ERROR(" track ar service not found !");
            value=false;   
        }
    }
    
    return (value);
}

//中心调度函数
void CENTRALCONTROLLING::execute(const oryxbot_msgs::centerGoalConstPtr &goal, center_server *as)
{
    oryxbot_msgs::centerFeedback feedback;
    oryxbot_msgs::centerResult result_;

    for(int i= goal->point_number; i<path.size()&&ros::ok(); i++)
    {
        oryxbot_msgs::nav_goal srv;
        srv.request.pose.x  =   path[i].point.pose.pose[0];
        srv.request.pose.y  =   path[i].point.pose.pose[1];
        srv.request.pose.theta  =   path[i].point.pose.pose[2];
        if(client_nav.call(srv))
        {
            suspend=~srv.response.success;
        }
        else
        {
            ROS_ERROR(" nav service not found !");
            suspend=true;            
        }
        stringstream ss;
        ss<< path[i].point_name << "  x:" << path[i].point.pose.pose[0] << "  y:" << path[i].point.pose.pose[1] << endl;
        feedback.message=ss.str();
        cout << ss.str() << endl;
        as->publishFeedback(feedback);//导航成功返回
        
        //执行action
        if(path[i].point.setp>0)
        {
            for(int j=0;j<path[i].point.setp;j++)
            {
                //暂停
                if(suspend==true){
                    cout << "zanting" <<endl;
                    while(suspend==true&&ros::ok())
                    {
                        sleep(1);
                    }
                }

                cout<< "fun : " << path[i].point.action[j].fun << " num : " << path[i].point.action[j].num ;
                if(!action(path[i].number,path[i].point.action[j].fun,path[i].point.action[j].num,path[i].point.action[j].parameter))
                {
                    ROS_ERROR("%s is error !",function_name[path[i].point.action[j].fun].c_str());
                    suspend=true;
                }
                else
                {
                    suspend=0;
                }
		        cout << "j: " << j <<endl;
            }//end for
		    cout << "end for" <<endl;
        }//end if

        //如果是环状路径，则继续执行第一个点；
        if(i==(path.size()-1)&&path[i].next=="ring")
        {i=-1; cout << "if ring "<< endl;}
        //在导航前暂停
        while(suspend==true&&ros::ok())
        {
	        cout << "zanting " << endl;
            sleep(1);
        }
        //sleep(1);
    }//end for
	cout << "end for ------" <<endl;
    as->setSucceeded(result_);
	cout << "end end end "<< endl;
}


//主函数
int main(int argc,char** argv)
{
    ros::init(argc,argv,"center");

    CENTRALCONTROLLING central;

    string path_str;
    path_str=getParam<string>("/center/PATH","/PATH1");
    readpath(path_str);
	//readpath("/PATH1");

    //开启多线程接收
    ros::AsyncSpinner spinner(3); 
    spinner.start();
    ros::waitForShutdown();
}
