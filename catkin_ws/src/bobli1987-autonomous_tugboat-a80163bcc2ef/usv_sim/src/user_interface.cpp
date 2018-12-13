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

#include "user_interface.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "usv_sim/WaypointTrackingAction.h"
#include "usv_sim/PathPlannerAction.h"
#include "waypoint_tracking_client.h"
#include "path_planner_client.h"
#include "maneuver.h"

// constructor
UserInterface::UserInterface()
{
  // define the publisher
  pub_path_ =
      nh_.advertise<visualization_msgs::Marker>("planner/path_marker", 1000);
  pub_goal_ =
      nh_.advertise<visualization_msgs::MarkerArray>("manager/goal_marker", 10);

  // initialize the path marker
  marker_path_.header.frame_id = "world";
  marker_path_.header.stamp = ros::Time::now();
  marker_path_.ns = "path/viz";
  marker_path_.action = visualization_msgs::Marker::ADD;
  marker_path_.pose.orientation.w = 1.0;
  marker_path_.id = 0;
  marker_path_.lifetime = ros::Duration(1);
  marker_path_.type = visualization_msgs::Marker::LINE_STRIP;
  marker_path_.scale.x = 0.05; // line width
  marker_path_.color.r = 0.0;
  marker_path_.color.g = 1.0;
  marker_path_.color.b = 0.0;
  marker_path_.color.a = 1.0;

  // initialize the goal marker
  marker_goal_.header.frame_id = "world";
  marker_goal_.header.stamp = ros::Time::now();
  marker_goal_.ns = "goal/viz";
  marker_goal_.action = visualization_msgs::Marker::ADD;
  marker_goal_.pose.orientation.w = 1.0;
  marker_goal_.id = 0;
  marker_goal_.lifetime = ros::Duration(1);
  marker_goal_.type = visualization_msgs::Marker::CYLINDER;
  marker_goal_.scale.x = 5;
  marker_goal_.scale.y = 5;
  marker_goal_.scale.z = 1;
  marker_goal_.color.r = 0.0;
  marker_goal_.color.g = 1.0;
  marker_goal_.color.b = 0.0;
  marker_goal_.color.a = 0.3;

  timer_ = nh_.createTimer(ros::Duration(1),
                           &UserInterface::timerCallback, this);
}

// callback of the timer
void UserInterface::timerCallback(const ros::TimerEvent& event)
{
  // publish path to rviz
  geometry_msgs::Point p;

  marker_path_.points.clear();
  marker_goal_array_.markers.clear();

  for (size_t i = 0; i != mission_.get_size(); ++i)
  {
    p.x = mission_.wpt_xcoor_[i];
    p.y = -mission_.wpt_ycoor_[i];
    p.z = 0;

    if (i == (mission_.get_size() - 1))
    {
      marker_goal_.id += 1;
      marker_goal_.pose.position.x = p.x;
      marker_goal_.pose.position.y = p.y;
      marker_goal_.pose.position.z = p.z;

      marker_goal_array_.markers.push_back(marker_goal_);
     }
     marker_path_.points.push_back(p);
   }

    // publish the messages
    pub_path_.publish(marker_path_);
    pub_goal_.publish(marker_goal_array_);
}


// generate new mission based on the target position
Maneuver UserInterface::GenerateMission(
    const double& start_xcoor, const double& start_ycoor,
    const double& start_psi, const double& goal_xcoor, const double& goal_ycoor,
    const double& goal_psi, const bool& station_keeping)
{
  // define the goal for the path planner
  usv_sim::PathPlannerGoal goal;

  // check the bound of the goal for the path planner
  double start_psi_temp = (start_psi < 0) ? (start_psi + 2 * M_PI) : start_psi;
  double end_psi_temp = (goal_psi < 0) ? (goal_psi + 2 * M_PI) : goal_psi;

  // prepare the goal for the path planner
  goal.start_pose = {start_xcoor, start_ycoor, start_psi_temp};
  goal.start_vel = {1.5, 0};
  goal.end_pose = {goal_xcoor, goal_ycoor, end_psi_temp};

  // compute a feasible path
  bool success = path_planner_client_.compute_path(goal);

  Maneuver mission;

  if (success)
  {
    mission.goal_xcoor_ = goal_xcoor;
    mission.goal_ycoor_ = goal_ycoor;
    mission.goal_psi_ = goal_psi;
    mission.wpt_xcoor_ = path_planner_client_.result_->wpt_x;
    mission.wpt_ycoor_ = path_planner_client_.result_->wpt_y;
    mission.wpt_heading_ = path_planner_client_.result_->wpt_psi;
    mission.wpt_xcoor_.pop_back();
    mission.wpt_ycoor_.pop_back();
    mission.wpt_heading_.pop_back();
    mission.wpt_xcoor_.push_back(goal_xcoor);
    mission.wpt_ycoor_.push_back(goal_ycoor);
    mission.wpt_heading_.push_back(goal_psi);

    mission.duration_ = ros::Duration(goal_duration_);
    mission.id_ = std::to_string(ros::Time::now().toSec());
    mission.station_keeping_on_ = station_keeping;

    if (station_keeping)
    {
      mission.description_ = "Transit to and keep position at (" +
                             std::to_string(int(goal_xcoor)) + ", " +
                             std::to_string(int(goal_ycoor)) + ", " +
                             std::to_string(int(goal_psi)) + ")";
    }
    else
    {
      mission.description_ = "Transit to  (" + std::to_string(int(goal_xcoor)) +
                             ", " + std::to_string(int(goal_ycoor)) + ")";
    }
  }

  return mission;
}

// run
int UserInterface::run()
{
  ros::Rate r(5);

  // define parameters for the ship's initial position
  const std::string PARAM_1 = "/ctrl_params/initialPosition/x";
  const std::string PARAM_2 = "/ctrl_params/initialPosition/y";
  const std::string PARAM_3 = "/ctrl_params/initialPosition/psi";

  double init_x, init_y, init_heading;

  bool ok_1 = ros::param::get(PARAM_1, init_x);
  bool ok_2 = ros::param::get(PARAM_2, init_y);
  bool ok_3 = ros::param::get(PARAM_3, init_heading);

  if (ok_1 && ok_2 && ok_3)
  {
    mission_ = GenerateMission(init_x, init_y, init_heading, goal_xcoor_, goal_ycoor_, goal_psi_, goal_station_keeping_);
    waypoint_tracking_client_.send_goal(&mission_);
    ROS_INFO("New maneuver is generated, and sent to the waypoint-tracking "
             "server.");
  }
  else
  {
    ROS_WARN("Initial position cannot be loaded. Use default values.");
  }

  while (ros::ok())
  {

    ros::spinOnce();
    r.sleep();
  }

  return EXIT_FAILURE;
}
