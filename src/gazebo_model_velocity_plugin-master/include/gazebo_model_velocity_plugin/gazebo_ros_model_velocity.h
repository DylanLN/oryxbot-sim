/*
 * Copyright 2015 Stefan Kohlbrecher, TU Darmstadt
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to exert
 *       forces on a robot, resulting in motion. Based on the
 *       planar_move plugin by Piyush Khandelwal.
 * Author: Stefan Kohlbrecher, Sammy Pfeiffer
 * Date: 06 August 2015, 21 December 2018
 */

#ifndef GAZEBO_ROS_MODEL_VELOCITY_HH
#define GAZEBO_ROS_MODEL_VELOCITY_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

#include <gazebo_model_velocity_plugin/speed_limiter.h>

namespace gazebo {

  class GazeboRosModelVelocity : public ModelPlugin {

    public: 
      GazeboRosModelVelocity();
      ~GazeboRosModelVelocity();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> nh_;
      ros::Subscriber vel_sub_;
      ros::Publisher output_vel_pub_;

      ros::Publisher odometry_pub_;
      ros::Publisher pose2d_pub;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      tf::Transform odom_transform_;
      ros::Time last_odom_publish_time_;
      double odometry_rate_;
      bool publish_odometry_tf_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      /// Gaussian noise
      double gaussian_noise_xy_;
      double gaussian_noise_yaw_;
      unsigned int seed;

      /// Gaussian noise generator
      double GaussianKernel(double mu, double sigma);

      void publishOdometry(double step_time);
      tf::Transform getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const;


      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string output_vel_topic_;
      double update_rate_;
      double command_timeout_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      // Latest command
      geometry_msgs::Twist current_cmd_;
      geometry_msgs::Twist last_cmd0_;
      geometry_msgs::Twist last_cmd1_;

      gazebo_model_velocity_plugin::SpeedLimiter limiter_lin_;
      gazebo_model_velocity_plugin::SpeedLimiter limiter_ang_;

      ros::Time last_velocity_update_time_;
      ros::Time last_command_time_;

  };

}

#endif /* end of include guard: GAZEBO_ROS_MODEL_VELOCITY_HH */
