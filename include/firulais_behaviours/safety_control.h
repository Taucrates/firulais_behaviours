/*
* This file is part of firulais_behaviours.
*
* Copyright (C) 2022 Antoni Tauler-Rosselló <a.tauler@uib.cat> (University of the Balearic Islands)
*
* firulais_behaviours is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* firulais_behaviours is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with firulais_behaviours. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCLUDE_FIRULAIS_BEHAVIOURS_SAFETY_CONTROL_H
#define INCLUDE_FIRULAIS_BEHAVIOURS_SAFETY_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <actionlib_msgs/GoalID.h>

#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/ControllerJoystick.h>
#include <unitree_legged_msgs/Ranges.h>

namespace firulais_behaviours {

class SafetyControl {
 public:
  explicit SafetyControl(const ros::NodeHandle& nh);
  virtual ~SafetyControl();

  void configure();

 private:
  // ROS variables
  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber firulais_high_state_;
  ros::Subscriber twist_sub_;  // ROS cmd_vel to control
  ros::Subscriber controller_sub_;
  ros::Subscriber front_range_;
  ros::Subscriber right_range_;
  ros::Subscriber left_range_;

  // Publishers
  ros::Publisher twist_pub_; // cmd_vel safety controlled
  ros::Publisher cancel_nav_;

  // Timers
  ros::Timer timer_;

  //tf::TransformBroadcaster tf_br_;
  //tf::TransformListener tf_lis_;

  //dynamic_reconfigure::Server<srv_mav_behaviours::safety_managerConfig> reconfigure_server_;

  //Services
  //ros::ServiceServer request_control_srv_;
  //ros::ServiceServer give_up_control_srv_;

  // Params
  double max_linear_speed;
  double max_yaw_speed;

  // Global variables
  geometry_msgs::Twist in_command_vel;
  //geometry_msgs::Twist filtered_command_vel;
  unitree_legged_msgs::HighState firulais_state;
  float ranges[3];

  bool cmd_vel_received = false;
  bool stop_autonomous = false;
  bool autonomous_stopped = false;
  ros::Time controller_time;

  // Services
  //bool requestControl(srv_mav_behaviours::RequestControl::Request &req, srv_mav_behaviours::RequestControl::Response &res);
  //bool giveUpControl(srv_mav_behaviours::GiveUpControl::Request &req, srv_mav_behaviours::GiveUpControl::Response &res);

  // Reconfigure
  //void dynReconfig(srv_mav_behaviours::safety_managerConfig &config, uint32_t level);

  // Callbacks
  void timerClb(const ros::TimerEvent& event);
  void inTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg);
  void highStateClb(const unitree_legged_msgs::HighState::ConstPtr& high_state_msg);
  void controllerClb(const unitree_legged_msgs::ControllerJoystick::ConstPtr& controller_msg);
  void frontRangeClb(const sensor_msgs::Range::ConstPtr& front_range_msg);
  void rightRangeClb(const sensor_msgs::Range::ConstPtr& right_range_msg);
  void leftRangeClb(const sensor_msgs::Range::ConstPtr& left_range_msg);

};

}  // namespace firulais_behaviours

#endif // INCLUDE_FIRULAIS_BEHAVIOURS_SAFETY_CONTROL_H