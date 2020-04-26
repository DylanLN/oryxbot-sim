#include "oryxbot_arm_controller.h"

Dobot_controller::Dobot_controller()
{
	ros::param::get("/oryxbot_arm_controller/port",port);
//	port="dobot";
	robot_init();
	set_ptp_params();

	set_zero_server = m_handle.advertiseService("set_zero",&Dobot_controller::set_zero_deal,this);
	get_zero_server = m_handle.advertiseService("get_zero",&Dobot_controller::get_zero_deal,this);
	pick_server = m_handle.advertiseService("pick",&Dobot_controller::pick_deal,this);
	place_server = m_handle.advertiseService("place",&Dobot_controller::place_deal,this);
	goto_position_server = m_handle.advertiseService("goto_position",&Dobot_controller::goto_position_deal,this);
	teach_server = m_handle.advertiseService("teach",&Dobot_controller::teach_deal,this);
	sucker_server = m_handle.advertiseService("sucker",&Dobot_controller::sucker_deal,this);

	position_sub = m_handle.subscribe("position_cmd",100,&Dobot_controller::position_callback,this);		
	position_pub = m_handle.advertise<oryxbot_msgs::dobot_control>("oryxbot_arm_controller/position_info",100);
	joint_pub = m_handle.advertise<sensor_msgs::JointState>("oryxbot_arm_controller/joint_states",100);
		
	pub_thread = boost::thread(boost::bind(&Dobot_controller::pub_func,this));

}

Dobot_controller::~Dobot_controller()
{

}

int Dobot_controller::robot_init()
{
	char name[20]="";
	SearchDobot(name,20);
	int result = ConnectDobot(port.c_str(),115200,0,0);
//	port.c_str()=name;
	cout << "port is " << port << endl;
	switch (result) {
		case DobotConnect_NoError:
			break;
		case DobotConnect_NotFound:
			ROS_ERROR("Dobot not found!");
			return -2;
			break;
		case DobotConnect_Occupied:
			ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
			return -3;
			break;
		default:
			break;
	}

	ROS_INFO("Dobot service running...");
	
	SetCmdTimeout(3000);
	SetQueuedCmdClear();
	SetQueuedCmdStartExec();
	uint8_t major_version ,minor_version ,revision;
	result = GetDeviceVersion(&major_version,&minor_version,&revision);
	if(result == DobotCommunicate_NoError){
	        ROS_INFO("Device version:%d.%d.%d", major_version, minor_version, revision);
	}else{
		ROS_INFO("Failed to get Dvice version information！");
	}

}

bool Dobot_controller::set_suctioncup(bool state)
{
      	uint64_t queuedCmdIndex;

    	int result = SetEndEffectorSuctionCup(true, state, false, &queuedCmdIndex);
	if(result == DobotCommunicate_NoError){
		return  true;
	}else{
		return false;
	}	

}

bool Dobot_controller::set_endeffector_params(float x,float y,float z)
{
	EndEffectorParams params;
	uint64_t queuedCmdIndex;
	
	params.xBias = x;
	params.yBias = y;
	params.zBias = z;
	
    	int result = SetEndEffectorParams(&params, true, &queuedCmdIndex);
	if(result == DobotCommunicate_NoError){
		return true;
	}else if(result == DobotCommunicate_BufferFull){
		ROS_INFO("SetEndEffectorparams 指令队列已满！");
		return false;
	}else if(result == DobotCommunicate_Timeout){
		ROS_INFO("SetEndEffectorparams 指令超时无返回！");
		return false;
	}

}

bool Dobot_controller::get_robot_pose(Pose* pose)
{
	int result = GetPose(pose);
	if(result == DobotCommunicate_NoError){
		return true;	
	}else{
		ROS_INFO("GetPose 指令超时无返回");
		return false;
	}

}

bool Dobot_controller::set_ptp_params()
{
	PTPJointParams joint_params;
   	PTPCoordinateParams coord_params;
   	PTPCommonParams common_params;
	uint64_t queuedCmdIndex;

	for(int i=0;i<4;i++){
		joint_params.velocity[i] = 100;
		joint_params.acceleration[i] = 100;
	}
	
	int result = SetPTPJointParams(&joint_params,true,&queuedCmdIndex);
	if(result == DobotCommunicate_BufferFull){
		ROS_INFO("SetPTPJointParams 指令队列已满！");
		return false;
	}else if(result == DobotCommunicate_Timeout){
		ROS_INFO("SetPTPJointParams 指令超时无返回！");
		return false;
	}

	coord_params.xyzVelocity = 100;
	coord_params.rVelocity = 100;
	coord_params.xyzAcceleration = 100;
	coord_params.rAcceleration = 100;
	
	result = SetPTPCoordinateParams(&coord_params,true,&queuedCmdIndex);	
	if(result == DobotCommunicate_BufferFull){
		ROS_INFO("SetPTPCoordinateParams 指令队列已满！");
		return false;
	}else if(result == DobotCommunicate_Timeout){
		ROS_INFO("SetPTPCoordinateParams 指令超时无返回！");
		return false;
	}

    	common_params.velocityRatio = 50;
    	common_params.accelerationRatio = 50;
    	result = SetPTPCommonParams(&common_params, true, &queuedCmdIndex);

	if(result == DobotCommunicate_BufferFull){
		ROS_INFO("SetPTPCommonParams 指令队列已满！");
		return false;
	}else if(result == DobotCommunicate_Timeout){
		ROS_INFO("SetPTPCommonParams. 指令超时无返回！");
		return false;
	}

	return  true;
}

bool Dobot_controller::set_robot_pose(vector<float> position)
{
    	PTPCmd cmd;
	uint64_t queuedCmdIndex;

    	cmd.ptpMode = 1;
    	cmd.x = position[0];
    	cmd.y = position[1];
    	cmd.z = position[2];
    	cmd.r = position[3];

    	int result = SetPTPCmd(&cmd, true, &queuedCmdIndex);	
        if(result == DobotCommunicate_BufferFull){
                ROS_INFO("SetPTPCmd 指令队列已满！");
                return false;
        }else if(result == DobotCommunicate_Timeout){
                ROS_INFO("SetPTPCmd 指令超时无返回！");
                return false;
        }
	

	return  true;	
}


void Dobot_controller::position_callback(const oryxbot_msgs::dobot_control msg)
{	
	vector<float> position_cmd;
	position_cmd.push_back(msg.x);
	position_cmd.push_back(msg.y);
	position_cmd.push_back(msg.z);
	position_cmd.push_back(msg.r);
	set_robot_pose(position_cmd);			
}

bool Dobot_controller::set_zero_deal(oryxbot_msgs::set_zero::Request &req, oryxbot_msgs::set_zero::Response &res)
{
	HOMEParams params;
	HOMECmd cmd;
	uint64_t queuedCmdIndex;
	
	params.x = req.x;
	params.y = req.y;
	params.z = req.z;
	params.r = req.r;	

	int result = SetHOMEParams(&params, false, &queuedCmdIndex);
	if (result != DobotCommunicate_NoError) {
		res.ret = false;
		return false;
	}

	result = SetHOMECmd(&cmd, false, &queuedCmdIndex);
	if (result == DobotCommunicate_NoError) {
		res.ret = true;
		return true;
	}else{
		res.ret = false;
		return  false;
	}

}

bool Dobot_controller::get_zero_deal(oryxbot_msgs::get_zero::Request &req, oryxbot_msgs::get_zero::Response &res)
{
  	HOMEParams params;
	int result = 0;

	if(req.query_flag == true){
		result = GetHOMEParams(&params);
		if (result == DobotCommunicate_NoError) {
			res.x = params.x;
			res.y = params.y;
			res.z = params.z;
			res.r = params.r;
			return  true;
		}
	}
	
	return false;	
}

bool Dobot_controller::pick_deal(oryxbot_msgs::pick_place::Request &req, oryxbot_msgs::pick_place::Response &res)
{
	vector<float> pick_position;	
	Pose current_position;

	pick_position.push_back(req.x);
	pick_position.push_back(req.y);
	pick_position.push_back(req.z);
	pick_position.push_back(req.r);
	
	if(!set_robot_pose(pick_position)) {
	    return false;
        }	
	//pick_position[2] -= 30;
	//set_robot_pose(pick_position);

	ros::Rate rate(100);
	uint64_t i=0;
	float data[3]={0};
	while(ros::ok()){
		i++;
		if(get_robot_pose(&current_position)){
			float a = fabs(current_position.x - pick_position[0]);
			float b = fabs(current_position.y - pick_position[1]);
			float c = fabs(current_position.z - pick_position[2]);
			float d = fabs(current_position.r - pick_position[3]);
			if(i%50==0){
				float line=pow(data[0]-a,2)+pow(data[1]-b,2)+pow(data[2]-c,2);
				if(line<10){res.ret = false;break;}		//长时间（0.5s）未移动退出
				data[0]=a;
				data[1]=b;
				data[2]=c;
			}
			if(a<0.05 && b<0.05 && c<0.05 && d<0.05){
				set_suctioncup(true);
				res.ret = true;
				break;	
			}
                        //ROS_INFO_STREAM("pick[" << a << " " << b << " " << c << " " << d << "]");
		}
	}
	
	
	return true;
	
}

bool Dobot_controller::place_deal(oryxbot_msgs::pick_place::Request &req, oryxbot_msgs::pick_place::Response &res)
{
	vector<float> place_position;
	Pose current_position;	

	place_position.push_back(req.x);
	place_position.push_back(req.y);
	place_position.push_back(req.z);
        //place_position.push_back(req.z+30);
	place_position.push_back(req.r);

	if(!set_robot_pose(place_position)) {
	    res.ret = false;
	    return false;
        }
	//place_position[2] -= 30;
	//set_robot_pose(place_position);
	
	ros::Rate rate(100);
	uint64_t i=0;
	float data[3]={0};
	while(ros::ok()){
		i++;
		if(get_robot_pose(&current_position)){
			float a = fabs(current_position.x - place_position[0]);
			float b = fabs(current_position.y - place_position[1]);
			float c = fabs(current_position.z - place_position[2]);
			float d = fabs(current_position.r - place_position[3]);
			if(i%50==0){
				float line=pow(data[0]-a,2)+pow(data[1]-b,2)+pow(data[2]-c,2);
				if(line<10){res.ret = false;break;}		//长时间（0.5s）未移动退出
				data[0]=a;
				data[1]=b;
				data[2]=c;
			}
			if(a<0.05 && b<0.05 && c<0.05 && d<0.05){
				set_suctioncup(false);
				res.ret = true;
				break;	
			}
                        //ROS_INFO_STREAM("pick[" << a << " " << b << " " << c << " " << d << "]");
		}
	}

		
	return true;

}

bool Dobot_controller::goto_position_deal(oryxbot_msgs::goto_position::Request &req, oryxbot_msgs::goto_position::Response &res)
{
        vector<float> set_position;
        set_position.push_back(req.x);
        set_position.push_back(req.y);
        set_position.push_back(req.z);
        set_position.push_back(req.r);
 
	if(!set_robot_pose(set_position)) {
	    res.ret = false;
	    return false;
        }

	Pose current_position;	

	ros::Rate rate(100);
	uint64_t i=0;
	float data[3]={0};
	while(ros::ok()){
		i++;
		if(get_robot_pose(&current_position)){
			float a = fabs(current_position.x - set_position[0]);
			float b = fabs(current_position.y - set_position[1]);
			float c = fabs(current_position.z - set_position[2]);
			float d = fabs(current_position.r - set_position[3]);
			if(i%50==0){
				float line=pow(data[0]-a,2)+pow(data[1]-b,2)+pow(data[2]-c,2);
				if(line<10){res.ret = false;break;}		//长时间（0.5s）未移动退出
				data[0]=a;
				data[1]=b;
				data[2]=c;
			}
			if(a<0.05 && b<0.05 && c<0.05 && d<0.05){
				res.ret = true;
				break;	
			}
                        //ROS_INFO_STREAM("pick[" << a << " " << b << " " << c << " " << d << "]");
		}
	}
	return true;

}


void Dobot_controller::pub_func()
{
	Pose query_position;
	arm_joint.name.resize(4);
	arm_joint.position.resize(4);
		
	ros::Rate loop(5);
	while(ros::ok()){
		if(get_robot_pose(&query_position)){
			pub_position.x = query_position.x;
			pub_position.y = query_position.y;
			pub_position.z = query_position.z;
			pub_position.r = query_position.r;

			arm_joint.header.stamp = ros::Time::now();
			arm_joint.name[0] = "joint_1";				
			arm_joint.name[1] = "joint_2";				
			arm_joint.name[2] = "joint_5";				
			arm_joint.name[3] = "joint_6";				
			arm_joint.position[0] = query_position.jointAngle[0];
			arm_joint.position[1] = query_position.jointAngle[1];
			arm_joint.position[2] = query_position.jointAngle[2];
			arm_joint.position[3] = query_position.jointAngle[3];

			position_pub.publish(pub_position);	
			joint_pub.publish(arm_joint);
		}
		ros::spinOnce();
		loop.sleep();
	}
}

////////////////////////////////////////////////////
//示教服务程序	
//mode : 0-退出示教 1-开始示教 2-执行第（number）次示教程序		number：mode=2时有用，执行第n次程序
vector<Pose> position_teach[10];
//获取示教线程
pthread_t teach_tid;
void *teach_fn(void* arg)
{
    int ret=*(int *)arg;
	Pose pose;
	bool trigered=false;
    while (ros::ok())
    {
        /* code for loop body */
        GetHHTTrigOutput(&trigered);
        if(trigered){
            GetPose(&pose);
			position_teach[ret].push_back(pose);
            //ROS_INFO("X: %f Y: %f Z: %f ",pose.x,pose.y,pose.z);
        }
    }
    return NULL;
}

//示教服务程序
int teach_count=0;
uint8_t teach_state=0;
bool Dobot_controller::teach_deal(oryxbot_msgs::standard_mode::Request &req,
									oryxbot_msgs::standard_mode::Response &res)
{
    ROS_INFO("is ok");
	if(req.mode==1&&teach_state==0)		//state=0可以进入示教，state=1示教状态
	{
		//示教模式选择
		SetHHTTrigMode(TriggeredOnPeriodicInterval);
		SetHHTTrigOutputEnabled(true);
		int err;
		err=pthread_create(&teach_tid,NULL,teach_fn,&teach_count);
      	if (err!=0){
			cout << "Error:unable to create thread," << err << endl;
			exit(-1);
      	}else{
			teach_count++;
			teach_state=1;
		}
	}else if(req.mode==0&&teach_state==1)
	{
		pthread_cancel(teach_tid);//关闭线程
		pthread_join(teach_tid,NULL);//等待线程退出
		ROS_INFO("TEACH FN IS EXIR :\n");
    	SetHHTTrigOutputEnabled(false);
		teach_state=0;
	}else if(req.mode==2){
		vector<Pose>::iterator pose_temp;
		for(pose_temp=position_teach[req.number].begin();pose_temp<position_teach[req.number].end();++pose_temp)
		{
			vector<float> set_position;
			set_position.push_back((*pose_temp).x);
			set_position.push_back((*pose_temp).y);
			set_position.push_back((*pose_temp).z);
			set_position.push_back((*pose_temp).r);
			ROS_INFO("X: %f Y: %f Z: %f ",set_position[0],set_position[1],set_position[2]);

			if(!set_robot_pose(set_position)) {
				break;
			}
			Pose current_position;	
			ros::Rate rate(100);
			uint64_t i=0;
			float data[3]={0};
			while(ros::ok()){
				i++;
				if(get_robot_pose(&current_position)){
					float a = fabs(current_position.x - set_position[0]);
					float b = fabs(current_position.y - set_position[1]);
					float c = fabs(current_position.z - set_position[2]);
					float d = fabs(current_position.r - set_position[3]);
					if(i%50==0){
						float line=pow(data[0]-a,2)+pow(data[1]-b,2)+pow(data[2]-c,2);
						if(line<10){break;}		//长时间（0.5s）未移动退出
						data[0]=a;
						data[1]=b;
						data[2]=c;
					}
					if(a<0.05 && b<0.05 && c<0.05 && d<0.05){
						break;	
					}
				}
			}
		}
	}
	res.success=true;
	return true;
}
////////////////////////////////////////////////////////

////////////////////////////////////////////////////////
//吸盘控制服务	只用到mode和success
bool Dobot_controller::sucker_deal(oryxbot_msgs::standard_mode::Request &req,
									oryxbot_msgs::standard_mode::Response &res)
{
	if(req.mode==1){
		set_suctioncup(true);
	}
	else if(req.mode==0){
		set_suctioncup(false);
	}
	res.success=true;
	return true;
}

////////////////////////////////////////////////////////

int main(int argc,char** argv)
{
	ros::init(argc,argv,"dboot");
	Dobot_controller  dobot_controller;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}
