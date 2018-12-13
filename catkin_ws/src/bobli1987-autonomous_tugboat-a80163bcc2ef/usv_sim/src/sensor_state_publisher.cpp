/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bo Li <lither@sjtu.edu.cn>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bo Li nor the names of its contributors may
 *     be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "usv_sim/Usv16State.h"
#include <geometry_msgs/Pose2D.h>

// declare the publishers and messages
ros::Publisher pub_vel, pub_pose;
geometry_msgs::Pose2D msg_pose;
geometry_msgs::Twist msg_vel;

// callback function of the subscriber to /ship/state
void SubCallback(const usv_sim::Usv16StateConstPtr& msg)
{
  msg_pose.x = msg->pose.x;
  msg_pose.y = msg->pose.y;
  msg_pose.theta = msg->pose.theta;
  msg_vel = msg->vel;

  // publish the messages
  pub_vel.publish(msg_vel);
  pub_pose.publish(msg_pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle nh;

  // define the subscriber
  ros::Subscriber sub_state = nh.subscribe("/ship/state", 10, &SubCallback);

  // define the publishers
  ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/ship/vel", 10);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose2D>("/ship/pose", 10);

  ros::spin();
}
