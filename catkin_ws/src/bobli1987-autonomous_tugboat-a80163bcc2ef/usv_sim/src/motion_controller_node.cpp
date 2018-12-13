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
#include "motion_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_controller");

  // control parameters
  path_following::control_params param_pf = {
      3.0, // ku
      0.5, // k1
      1.0, // k2
      0.2, // u_acc_max
      0.5, // k_acc_max
      1.0  // u_d_yaw
  };

  path_following::vehicle_params param_pf_v = {
      Eigen::Vector3d(207, 217, 343.2345), // M
      Eigen::Vector3d(90, 300, 400)        // D
  };

  station_keeping::control_params param_sk = {
      Eigen::Vector3d(10, 10, 100),      // kp
      Eigen::Vector3d(400, 400, 60),     // kd
      Eigen::Vector3d(0.05, 0.05, 0.05), // nl_scale
      Eigen::Vector3d(0.16, 0.16, 0.16)  // lambda
  };

  station_keeping::vehicle_params param_sk_v = {
      Eigen::Vector3d(207, 217, 343.2345), // M
      Eigen::Vector3d(56.2, 300, 400)      // D
  };

  MotionController controller(2.0, param_pf_v, param_pf, param_sk_v, param_sk,
                              0.1);

  return controller.run();
}
