#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "oryxbot_msgs/SetArPose.h"
#include "oryxbot_msgs/SetRelativeMove.h"
#include "oryxbot_msgs/SetCharge.h"
#include <tf/transform_broadcaster.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool charged_flag;

double anchor_x,anchor_y, anchor_theta, anchor_qx, anchor_qy, anchor_qz, anchor_qw;

ros::ServiceClient PoseAdjustClient;
ros::ServiceClient RelativeMoveClient;
ros::ServiceClient TrackClient;
MoveBaseClient* ac;

bool arrival_check(){
	charged_flag = true;
}

//回到充电桩
bool backoff(float relative_move=0.19){
	oryxbot_msgs::SetRelativeMove relative_srv;
	relative_srv.request.mode = 0;
	relative_srv.request.relative_move = relative_move;
	if(!RelativeMoveClient.call(relative_srv)){
		return false;
		ROS_INFO("charging. . . . .");
	}
        if(!relative_srv.response.success)
		return false;
        ROS_INFO("back off ...");
	return true;
}
//追踪ar码
bool adjust(int mode=127){
        oryxbot_msgs::SetCharge srv;
        //选择跟踪ar码所含有的信息
        srv.request.charge=mode;
        if(!TrackClient.call(srv)){
		return false;	
        }
	if(!srv.response.success)
		return false;

	oryxbot_msgs::SetArPose adjust_srv;
	adjust_srv.request.adjust = true;
	if(!PoseAdjustClient.call(adjust_srv)){
                srv.request.charge=127;
                if(TrackClient.call(srv)){
                        return false;	
                }
		return false;	
	}

        srv.request.charge=127;
        //ar码信息回归127
        if(!TrackClient.call(srv)){
		return false;	
        }
	if(!srv.response.success )
		return false;
        
        return true;
}

//导航到目标点
bool nav2anchor() {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(anchor_theta);
      goal.target_pose.pose.position.x = anchor_x;
      goal.target_pose.pose.position.y = anchor_y;
      goal.target_pose.pose.orientation.x = orientation.x;
      goal.target_pose.pose.orientation.y = orientation.y;
      goal.target_pose.pose.orientation.z = orientation.z;
      goal.target_pose.pose.orientation.w = orientation.w;

      ROS_INFO("going to anchor position to charge");
      ac->sendGoal(goal);
      ac->waitForResult();
      if(ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("going to anchor positon fialed, charging faield");
	  return false;
      }
      ROS_INFO("arrival_at anchor posistio");
      return true;
}

bool TrackCallback(oryxbot_msgs::SetRelativeMove::Request &req, 
                   oryxbot_msgs::SetRelativeMove::Response&res)
{
        if (req.mode>=0 && req.mode<127)
        {
                /* code for True */
                if(req.relative_move>0.0){
                        if(!adjust(req.mode)) {
                                res.message = "adjust failed";
                                res.success =  false; 
                                return true; 
                        }
                        if(!backoff(0.19-req.relative_move)){
                                res.message = "backoff faield";
                                res.success =  false; 
                                return true; 
                        }

                        res.message =  "track success";
                        res.success =  true;
                        return true;
                }else{
                        res.message =  "Out of range";
                        res.success =  false;
                        return true;
                }
        }else{
                res.message =  "The AR code is invalid";
                res.success =  false;
                return true;
        }
}

bool ChargeCallback(oryxbot_msgs::SetCharge::Request &req, oryxbot_msgs::SetCharge::Response&res){
	if (req.charge==1){
                if(!nav2anchor()) {
                        res.message = "nav to anchor failed";
                        res.success =  false;  
			return true;
                }
		sleep(1);
		if(!adjust(0)) {
                        res.message = "adjust failed";
                        res.success =  false; 
			return true; 
                }
		if(!backoff()){
                        res.message = "backoff faield";
                        res.success =  false; 
			return true; 
                }
			
	}else if(req.charge==2){
	                if(!nav2anchor()) {
                        res.message = "nav to anchor failed";
                        res.success =  false;
                        return true;
                }

	}else if(req.charge==3){
                if(!adjust(0)) {
                        res.message = "adjust failed";
                        res.success =  false;
                        return true;
                }
                if(!backoff()){
                        res.message = "backoff faield";
                        res.success =  false;
                        return true;
                }
	}
        res.message =  "charge success";
        res.success =  true;
        return true;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "auto_charging");
	ros::NodeHandle n;
        ros::NodeHandle private_nh("~");
	
        ros::ServiceServer AutoChargingServer = n.advertiseService("goto_charge", ChargeCallback);
        ros::ServiceServer Track_Server = n.advertiseService("track_ar", TrackCallback);

	PoseAdjustClient = n.serviceClient<oryxbot_msgs::SetArPose>("/pose_adjust/adjust_service");
	TrackClient = n.serviceClient<oryxbot_msgs::SetCharge>("Track");

        RelativeMoveClient = n.serviceClient<oryxbot_msgs::SetRelativeMove>("/relative_move");
		        
        ac = new MoveBaseClient ("move_base", true);
        while(!ac->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
	private_nh.param<double>("anchor_x", anchor_x, 0);
        private_nh.param<double>("anchor_y", anchor_y, 0);
        private_nh.param<double>("anchor_theta", anchor_theta, 0);

	ros::spin();
        delete ac;
	return 0;
}

