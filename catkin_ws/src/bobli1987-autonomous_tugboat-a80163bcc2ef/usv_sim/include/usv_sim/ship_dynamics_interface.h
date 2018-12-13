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

#ifndef SHIP_DYNAMICS_INTERFACE_H
#define SHIP_DYNAMICS_INTERFACE_H

#include <ros/ros.h>
#include <memory>
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include "usv16.h"
#include <geometry_msgs/Pose2D.h>
#include "usv_sim/control.h"

class ShipDynamicsInterface
{

public:
  /////////////////
  // Constructor //
  /////////////////

  ShipDynamicsInterface(const double& timer_step);

  ///////////////////
  // Public Method //
  ///////////////////

  int run()
  {
    ros::spin();
    return EXIT_FAILURE;
  }

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // ros node
  ros::NodeHandle nh_;

  // timer which controls simulatoin and the publishing rate
  ros::Timer timer_;

  // publishers and messages
  std::unique_ptr<ros::Publisher> pubPtr_vel_;
  std::unique_ptr<ros::Publisher> pubPtr_pos_;
  geometry_msgs::Twist msg_vel_;
  geometry_msgs::Pose2D msg_pos_;

  // subscriber
  std::unique_ptr<ros::Subscriber> subPtr_act_;

  // vehicle for the simulation
  std::unique_ptr<USV16> Ptr_vehicle_;

  // time step of message publishing
  double step_size_ = 0.05;

  // flag
  bool received_control_;

  /////////////////////
  // Private Methods //
  /////////////////////

  void get_ros_param();
  void timerCallback(const ros::TimerEvent&);
  void callback_sub(const usv_sim::control& msg_ctrl);
};

#endif // SHIP_DYNAMICS_INTERFACE_H
