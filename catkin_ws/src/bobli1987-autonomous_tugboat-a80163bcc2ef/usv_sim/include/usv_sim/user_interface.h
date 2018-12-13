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

#ifndef TASK_USER_INTERFACE_H
#define TASK_USER_INTERFACE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "maneuver.h"
#include "waypoint_tracking_client.h"
#include "path_planner_client.h"
#include "usv_sim/WaypointTrackingAction.h"
#include "usv_sim/obstacle.h"

class UserInterface
{
private:
  ////////////////////
  // Private Fields //
  ////////////////////

  ros::NodeHandle nh_;
  ros::ServiceServer srv_user_;
  ros::ServiceClient srv_obstacle_;
  ros::Timer timer_;

  // publisher
  ros::Publisher pub_path_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_target_text_;

  // rviz visualization msg
  visualization_msgs::Marker marker_path_;
  visualization_msgs::Marker marker_goal_;
  visualization_msgs::MarkerArray marker_goal_array_;

  // command inputs from the user
  double goal_xcoor_ = 200;
  double goal_ycoor_ = 300;
  double goal_psi_ = 0;
  bool goal_station_keeping_ = false;
  double goal_duration_ = 600;

  // actionlib clients
  WaypointTrackingClient waypoint_tracking_client_{"guidance_node"};
  PathPlannerClient path_planner_client_{"planner_node"};
  
  // radius of a new obstacle
  double obstacle_radius_ = 5;

  // mission
  Maneuver mission_;

  ////////////////////
  // Private Method //
  ////////////////////

  void timerCallback(const ros::TimerEvent&);

  // generate new mission based on the goal position
  Maneuver GenerateMission(const double& start_xcoor, const double& start_ycoor,
                           const double& start_psi, const double& goal_xcoor,
                           const double& goal_ycoor, const double& goal_psi,
                           const bool& goal_station_keeping);

public:
  /////////////////
  // Constructor //
  /////////////////

  UserInterface();

  ////////////////////
  // Public Methods //
  ////////////////////

  // run
  int run();
};

#endif // TASK_USER_INTERFACE_H
