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

#include "path_planner_server.h"

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include "usv_sim/PathPlannerAction.h"

#include "path_planner/ConfigFile.h"
#include "path_planner/TrajectoryPlanner.h"
#include "path_planner/Parser.h"
#include "path_planner/Action.h"

using namespace std;

/////////////////
// Constructor //
/////////////////

PathPlannerServer::PathPlannerServer(std::string name)
    : as_(nh_, name, boost::bind(&PathPlannerServer::executeCallback, this, _1),
          false)
{
  // define the service server
  srv_obstacle_ = nh_.advertiseService(
      "/obstacle_update", &PathPlannerServer::obstacleCallback, this);

  // define the publisher
  pub_markerArray_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/static_obstacles", 1000);

  // the timer to control the publishing of obstacles to rviz
  timer_step_ = 1;
  timer_ = nh_.createTimer(ros::Duration(timer_step_),
                           &PathPlannerServer::timerCallback, this);

  // define the waypoint labels
  markerArray_obstacle_.markers.clear();

  // load the obstacles
  Parser lParserObj;

  std::string ObstacleFile = "/home/jiangxvv/catkin_ws/src/bobli1987-autonomous_tugboat-a80163bcc2ef/usv_sim/config/obstacles/ObstaclesCenter.txt";
  std::ifstream f(ObstacleFile.c_str());
  if (!f.good())
    ROS_ERROR_STREAM("ObstacleCenter file cannot be found!");
  else
  {
    ROS_INFO_STREAM("ObstacleCenter file is found.");

    obstacles_ = lParserObj.loadObstaclesCenter(ObstacleFile);
    
    marker_cylinder_.header.frame_id = "world";
    marker_cylinder_.header.stamp = ros::Time::now();
    marker_cylinder_.ns = "static_obstacles/viz";
    marker_cylinder_.action = visualization_msgs::Marker::ADD;
    marker_cylinder_.type = visualization_msgs::Marker::CYLINDER;
    marker_cylinder_.lifetime = ros::Duration(timer_step_);

    for (uint32_t i = 0; i < obstacles_.size(); ++i)
    {    
      marker_cylinder_.id = i + 2;     
      marker_cylinder_.scale.x = 2 * obstacles_[i][2];
      marker_cylinder_.scale.y = 2 * obstacles_[i][2];
      marker_cylinder_.scale.z = 1;
      marker_cylinder_.pose.position.x = obstacles_[i][0];
      marker_cylinder_.pose.position.y = -obstacles_[i][1];
      marker_cylinder_.pose.position.z = 0.0;
      marker_cylinder_.color.r = 1.0; // red
      marker_cylinder_.color.b = 0.0;
      marker_cylinder_.color.g = 0.0;
      marker_cylinder_.color.a = 0.5;

      markerArray_obstacle_.markers.push_back(marker_cylinder_);
    }
    
    std::string path = "/home/jiangxvv/catkin_ws/src/bobli1987-autonomous_tugboat-a80163bcc2ef/usv_sim/config";
    lPlanner_ = std::unique_ptr<TrajectoryPlanner>(new TrajectoryPlanner(path));
  }

  // start the action server
  as_.start();
  ROS_INFO("Path planner starts, waiting for a goal.");
}

/////////////////////
// Private Methods //
/////////////////////

void PathPlannerServer::executeCallback(
    const usv_sim::PathPlannerGoalConstPtr& goal)
{  
  vector<double> start_pose = goal->start_pose;
  vector<double> end_pose = goal->end_pose;
  vector<double> start_vel = goal->start_vel;
 
  vector<double> lTraj =
      lPlanner_->computeTrajectory(start_pose, start_vel, end_pose);

  usv_sim::PathPlannerResult result;

  if (lTraj.size() < 1)
  {
    ROS_ERROR("Cannot not find a trajectory...!!!");
    as_.setAborted(result);
  }
  else
  {
    for (size_t i = 0; i < lTraj.size() / 3; ++i)
    {
      result.wpt_x.push_back(lTraj.at(3 * i));
      result.wpt_y.push_back(lTraj.at(3 * i + 1));
      result.wpt_psi.push_back(lTraj.at(3 * i + 2));
    }

    // set the action state to succeeded
    ROS_INFO("A trajectory is found!!!");
    as_.setSucceeded(result);
  }
}

// callback of the timer
void PathPlannerServer::timerCallback(const ros::TimerEvent& event)
{
  pub_markerArray_.publish(markerArray_obstacle_);
}

// callback of the service server
bool PathPlannerServer::obstacleCallback(usv_sim::obstacle::Request& req,
                                         usv_sim::obstacle::Response& resp)
{
  resp.response = "Path planner received a new obstacle.";
  
  vector<double> obstacle = {req.x_coor, req.y_coor, req.radius};
  
  // send the new static obstacle to the path planner
  lPlanner_->addStaticObstacles(obstacle);
  obstacles_.push_back(obstacle);
  
  marker_cylinder_.id = obstacles_.size() + 2;
  marker_cylinder_.scale.x = 2 * obstacle[2];
  marker_cylinder_.scale.y = 2 * obstacle[2];
  marker_cylinder_.scale.z = 1;
  marker_cylinder_.pose.position.x = obstacle[0];
  marker_cylinder_.pose.position.y = -obstacle[1];
  marker_cylinder_.pose.position.z = 0.0;
  marker_cylinder_.color.r = 1.0; // red
  marker_cylinder_.color.b = 0.0;
  marker_cylinder_.color.g = 0.0;
  marker_cylinder_.color.a = 0.5;

  markerArray_obstacle_.markers.push_back(marker_cylinder_);
  
  return true;
}
