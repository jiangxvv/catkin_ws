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

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Dense>
#include "pf_controller.h"
#include "sk_controller.h"
#include "thrust_allocation_law.h"
#include "usv_sim/course.h"
#include "usv_sim/control.h"

class MotionController
{
public:
  /////////////////
  // Constructor //
  /////////////////

  MotionController(const double& u_d,
                   const path_following::vehicle_params& pf_v,
                   const path_following::control_params& pf_c,
                   const station_keeping::vehicle_params& sk_v,
                   const station_keeping::control_params& sk_c,
                   const double& time_step);

  ////////////////////
  // Public Methods //
  ////////////////////

  int run()
  {
    ros::spin();

    return EXIT_FAILURE;
    ;
  }

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // ros node
  ros::NodeHandle nh_;

  // publisher and message of the actuation signal
  std::unique_ptr<ros::Publisher> pubPtr_control_;
  usv_sim::control msg_actuation_;

  // create a timer to control the publishing of control commands
  ros::Timer timer_;

  // subscribers
  std::unique_ptr<ros::Subscriber> sub_pos_;
  std::unique_ptr<ros::Subscriber> sub_vel_;
  std::unique_ptr<ros::Subscriber> sub_course_;  // for path following
  std::unique_ptr<ros::Subscriber> sub_station_; // for station keeping

  // actuation computed by the controller
  Eigen::Vector3d actuation_;

  // the time step of the controller
  double step_size_ = 0.1;

  // variables used for the controllers
  double u_, v_, r_;
  double x_, y_, psi_;
  double r_d_, dr_d_, psi_d_;
  double x_d_, y_d_;

  // helper variable
  bool received_course_info_ = false;
  bool received_station_info_ = false;

  // path-following controller
  double u_d_; // desired surge speed
  path_following::PFController pf_controller_;

  // station-keeping controller
  station_keeping::SKController sk_controller_;

  /////////////////////
  // Private Methods //
  /////////////////////

  void callback_pose(const geometry_msgs::Pose2DConstPtr& msg_pose)
  {
    x_ = msg_pose->x;
    y_ = msg_pose->y;
    psi_ = msg_pose->theta;
  }

  void callback_vel(const geometry_msgs::Twist& msg_vel)
  {
    u_ = msg_vel.linear.x;
    v_ = msg_vel.linear.y;
    r_ = msg_vel.angular.z;
  }

  void callback_course(const usv_sim::course& msg_course)
  {
    psi_d_ = msg_course.angle;
    r_d_ = msg_course.rate;
    dr_d_ = msg_course.acceleration;

    received_course_info_ = true;
  }

  void callback_station(const geometry_msgs::Pose2DConstPtr& msg_station)
  {
    x_d_ = msg_station->x;
    y_d_ = msg_station->y;
    psi_d_ = msg_station->theta;

    received_station_info_ = true;
  }

  void timerCallback(const ros::TimerEvent&);
};

#endif // MOTION_CONTROLLER_H
