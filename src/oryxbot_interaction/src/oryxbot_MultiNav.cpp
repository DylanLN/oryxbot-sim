#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <ros/ros.h>
#include <oryxbot_msgs/nav_goal.h>

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
//point类型
typedef struct {
    float pose[3];
    string name;
}Point;

vector<Point> point;//多点集合

//读取点位函数
//以下是point参数所需str
const string param_str="/oryxbot_MultiNav";
    const string point_str="/point";
        const string name_str="/name";        
        const string pose_str="/pose";
            const string xyth_str[3]={"/x","/y","/th"};
//读取所有点
int readpoint(void)
{
  point.clear();
  stringstream ss;
  string str="";

  int count=0;
  while(ros::ok()){
    stringstream point_name;
    point_name.clear();
    point_name<<point_str<<count;
    ss<<param_str<<point_name.str()<<pose_str<<xyth_str[0];
    ss>>str;
    const string &str0=str;
    float param = getParam<float>(str0,10000.0);
    if(param==10000.0){
      break;
    }else{
      count++;
    }
    str="";
    ss.str("");
    ss.clear();
  }

  for(int i=0;i<count;i++){
    Point p;
    stringstream point_name;
    point_name.clear();
    point_name<<point_str<<i;
    //name
    if(true){
      ss<<param_str<<point_name.str()<<name_str;
      ss>>str;
      const string &str1=str;
      p.name = getParam<string>(str1,"");
      str="";
      ss.str("");
      ss.clear();
    }
    //pose
    for(int j=0;j<3;j++){
        ss<<param_str<<point_name.str()<<pose_str<<xyth_str[j];
        ss>>str;
        const string &str0=str;
        p.pose[j]=getParam<float>(str0,0.0);
        str="";
        ss.str("");
        ss.clear();
    }
    point.push_back(p);
  }
  return count;
}

//主函数
int main(int argc,char** argv)
{
    ros::init(argc,argv,"oryxbot_MultiNav");
    ros::NodeHandle nh;
    //客户端
    ros::ServiceClient client_nav = nh.serviceClient<oryxbot_msgs::nav_goal>("oryxbot_navgoali");//导航
    //读取路径
    int count=readpoint();
    //开始调用
    for(int i=0;i<count;i++){
      	ROS_INFO("nav go");
        oryxbot_msgs::nav_goal srv;
        srv.request.pose.x  =   point[i].pose[0];
        srv.request.pose.y  =   point[i].pose[1];
        srv.request.pose.theta  =   point[i].pose[2];
        if(!client_nav.call(srv)){
            ROS_ERROR(" nav service not found !");
        }
        stringstream ss;
        ss<< point[i].name << "  x:" <<  point[i].pose[0] << "  y:" <<  point[i].pose[1] << " th:" <<  point[i].pose[3] <<endl;
        cout << ss.str() << endl;        
    }
    return 0;
}
