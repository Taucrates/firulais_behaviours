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

#include "firulais_behaviours/safety_control.h"

namespace firulais_behaviours{

SafetyControl::SafetyControl(const ros::NodeHandle& nh) :
  nh_(nh)
{

  configure();

}

SafetyControl::~SafetyControl(){
}

void SafetyControl::configure(){
  

  nh_.param("max_linear_speed", max_linear_speed, 0.6);
  ROS_INFO("Max_linear_speed: %2.2f", max_linear_speed);

  nh_.param("max_yaw_speed", max_yaw_speed, 1.0);
  ROS_INFO("Max_yaw_speed: %2.2f", max_yaw_speed);

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist_out", 1);
  cancel_nav_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

  // Subscribers
  twist_sub_ = nh_.subscribe("twist_in", 1, &SafetyControl::inTwistClb, this);
  firulais_high_state_ = nh_.subscribe("high_state", 1, &SafetyControl::highStateClb, this);
  controller_sub_ = nh_.subscribe("/controller", 1, &SafetyControl::controllerClb, this);
  front_range_ = nh_.subscribe("front_range", 1, &SafetyControl::frontRangeClb, this);
  right_range_ = nh_.subscribe("right_range", 1, &SafetyControl::rightRangeClb, this);
  left_range_ = nh_.subscribe("left_range", 1, &SafetyControl::leftRangeClb, this);

  // Timers
  timer_ = nh_.createTimer(ros::Duration(1.0 / 30.0), &SafetyControl::timerClb, this);

  controller_time = ros::Time::now();

}

// Callbacks

void SafetyControl::timerClb(const ros::TimerEvent& event){

  geometry_msgs::Twist filtered_command_vel;

  if(!stop_autonomous){

    filtered_command_vel = in_command_vel;

    if(abs(filtered_command_vel.linear.x) > abs(max_linear_speed)){

      if(filtered_command_vel.linear.x > 0.0){
        filtered_command_vel.linear.x = max_linear_speed;
      } else {
        filtered_command_vel.linear.x = -max_linear_speed;
      }
      
    }

    if(abs(filtered_command_vel.linear.y) > abs(max_linear_speed)){

      if(filtered_command_vel.linear.y > 0.0){
        filtered_command_vel.linear.y = max_linear_speed;
      } else {
        filtered_command_vel.linear.y = -max_linear_speed;
      }
      
    }
    
    if(abs(filtered_command_vel.angular.z) > abs(max_yaw_speed)){

      if(filtered_command_vel.angular.z > 0.0){
        filtered_command_vel.angular.z = max_yaw_speed;
      } else {
        filtered_command_vel.angular.z = -max_yaw_speed;
      }
      
    }

    if(ranges[0] < 0.25){
      filtered_command_vel.linear.y -= abs(in_command_vel.linear.y);
    }

    if(ranges[1] < 0.25){
      filtered_command_vel.linear.x -= abs(in_command_vel.linear.x);
    }

    if(ranges[2] < 0.25){
      filtered_command_vel.linear.y += abs(in_command_vel.linear.y);
    }

    twist_pub_.publish(filtered_command_vel);

  } else if(!autonomous_stopped){

    ROS_WARN("Autonomous behavior stopped!");

    // Cancel navigation goal
    actionlib_msgs::GoalID cancel_msg;
    cancel_nav_.publish(cancel_msg);
    twist_pub_.publish(filtered_command_vel);

    autonomous_stopped = true;

  }

  //printf("Left: %3.2f, Frontal: %3.2f, Right: %3.2f\n", ranges[0], ranges[1], ranges[2]);

}

void SafetyControl::inTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg){

  in_command_vel = *twist_msg;
  cmd_vel_received = true;

}

void SafetyControl::highStateClb(const unitree_legged_msgs::HighState::ConstPtr& high_state_msg){

  firulais_state = *high_state_msg;

  // if(firulais_state.wirelessRemote[2] == 48 && stop_autonomous && (ros::Time::now().toSec() - controller_time.toSec()) > 2.0){ // Quick R2 + up to enable autonomous behaviour.
  //   ROS_WARN("Autonomous behaviour enabled.");
  //   stop_autonomous = false;
  //   autonomous_stopped = false;
  //   controller_time = ros::Time::now();
  // } else if((firulais_state.wirelessRemote[2] != 0 || firulais_state.wirelessRemote[3] != 0) && firulais_state.mode == 2 && (ros::Time::now().toSec() - controller_time.toSec()) > 2.0){
    
  //   stop_autonomous = true;
  // } 
    

}

void SafetyControl::controllerClb(const unitree_legged_msgs::ControllerJoystick::ConstPtr& controller_msg){
  
  if(controller_msg->stSelRsLs == 48 && stop_autonomous && (ros::Time::now().toSec() - controller_time.toSec()) > 2.0){ // Quick R2 + up to enable autonomous behaviour.
    ROS_WARN("Autonomous behaviour enabled.");
    stop_autonomous = false;
    autonomous_stopped = false;
    controller_time = ros::Time::now();
  } else if((abs(controller_msg->leftFrontal) > 0.05 || abs(controller_msg->leftLateral) > 0.05 || abs(controller_msg->rightFrontal) > 0.05 || abs(controller_msg->rightLateral) > 0.05) && cmd_vel_received && firulais_state.mode == 2 && (ros::Time::now().toSec() - controller_time.toSec()) > 2.0){
    printf("HOLA\n");
    stop_autonomous = true;
    cmd_vel_received = false;
  } 
    

}


void SafetyControl::frontRangeClb(const sensor_msgs::Range::ConstPtr& front_range_msg){

  ranges[1] = front_range_msg->range;

}

void SafetyControl::rightRangeClb(const sensor_msgs::Range::ConstPtr& right_range_msg){

  ranges[2] = right_range_msg->range;

}

void SafetyControl::leftRangeClb(const sensor_msgs::Range::ConstPtr& left_range_msg){

  ranges[0] = left_range_msg->range;

}


/*********************************************************************************************************************************************
************************************************************ METHODS *************************************************************************
**********************************************************************************************************************************************/

} // namespace