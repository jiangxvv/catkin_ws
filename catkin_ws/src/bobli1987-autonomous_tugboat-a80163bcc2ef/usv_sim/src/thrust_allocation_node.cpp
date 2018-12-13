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
#include "thrust_allocation_law.h"
#include "usv_sim/actuator.h"
#include "usv_sim/control.h"

ros::Publisher pub_actuator;
ros::Publisher pub_actuation;

// messages
usv_sim::actuator msg_actuator;
usv_sim::control msg_actuation;

// thrust allocaton law
thrust_allocaton_params ta_param = {
    120, // T_fwd
    100, // T_rev
    2,   // lx
    1    // ly
};

ThrustAllocation thrust_allocation(ta_param.T_fwd, ta_param.T_rev, ta_param.lx,
                                   ta_param.ly);

// input of thrust allocation law
std::array<double, 3> ta_input = {0, 0, 0};

// output of thrust allocation law
std::array<double, 7> output = {0, 0, 0, 0, 0, 0, 0};

void SubCallback(const usv_sim::controlConstPtr& actuation)
{
  // set the control signal
  ta_input[0] = actuation->surge;
  ta_input[1] = actuation->sway;
  ta_input[2] = actuation->yaw;

  // thrust allocation
  output = thrust_allocation.ComputeOutput(ta_input);

  // set the actuation signal
  msg_actuation.surge = output[4];
  msg_actuation.sway = output[5];
  msg_actuation.yaw = output[6];

  // set the actuator commmand signal
  msg_actuator.T_p = output[0];
  msg_actuator.T_s = output[2];
  msg_actuator.Alpha_p = (output[0] != 0)
                             ? atan(output[1] / output[0]) * 180 / M_PI
                             : 0; // in degree
  msg_actuator.Alpha_s = (output[2] != 0)
                             ? atan(output[3] / output[2]) * 180 / M_PI
                             : 0; // in degree

  // publish the messages
  pub_actuation.publish(msg_actuation);
  pub_actuator.publish(msg_actuator);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thrust_allocation");
  ros::NodeHandle nh;

  pub_actuation = nh.advertise<usv_sim::control>("ship/actuation", 1000);
  pub_actuator = nh.advertise<usv_sim::actuator>("ship/actuator_cmd", 1000);
  ros::Subscriber sub_actuation =
      nh.subscribe("ship/auto_control", 1000, &SubCallback);

  ros::spin();

  return 0;
}
