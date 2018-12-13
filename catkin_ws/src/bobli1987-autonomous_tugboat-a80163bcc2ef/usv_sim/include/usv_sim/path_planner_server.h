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

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/MarkerArray.h>
#include "usv_sim/PathPlannerAction.h"
#include "usv_sim/obstacle.h"
#include "path_planner/TrajectoryPlanner.h"

class PathPlannerServer
{
protected:
  //////////////////////
  // Protected Fields //
  //////////////////////

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<usv_sim::PathPlannerAction> as_;

  // service server responding to new obstacles
  ros::ServiceServer srv_obstacle_;

  // publishers
  ros::Publisher pub_markerArray_;

  // messages
  visualization_msgs::MarkerArray markerArray_obstacle_;

  // timer which controls the publication of marker array to rviz
  ros::Timer timer_;

  // time step of the timer
  double timer_step_;

  // generated waypoints
  std::vector<double> waypoints_xcoor_;
  std::vector<double> waypoints_ycoor_;
  std::vector<double> waypoints_heading_;

  // obstacles
  std::vector<std::vector<double>> obstacles_;
  
  // define the trajectory path_planner
  std::unique_ptr<TrajectoryPlanner> lPlanner_;
  
  // marker of obstacles
  visualization_msgs::Marker marker_cylinder_;

public:
  /////////////////
  // Constructor //
  /////////////////

  PathPlannerServer(std::string);

  ////////////////
  // Destructor //
  ////////////////
  

  ~PathPlannerServer(void) {}

private:
  /////////////////////
  // Private Methods //
  /////////////////////

  void executeCallback(const usv_sim::PathPlannerGoalConstPtr& goal);
  void timerCallback(const ros::TimerEvent& event);
  bool obstacleCallback(usv_sim::obstacle::Request&, usv_sim::obstacle::Response&);
};

#endif // PATH_PLANNER_H
