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

#ifndef PATH_PLANNER_CLIENT_H
#define PATH_PLANNER_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "usv_sim/PathPlannerAction.h"
#include <geometry_msgs/Pose2D.h>
#include "maneuver.h"

class PathPlannerClient
{
  friend class UserInterface;

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // simple action client
  actionlib::SimpleActionClient<usv_sim::PathPlannerAction> ac_;

  // result
  usv_sim::PathPlannerResultConstPtr result_;

public:
  /////////////////
  // Constructor //
  /////////////////

  PathPlannerClient(const std::string& server_name) : ac_(server_name, true)
  {
    ROS_INFO("Waiting for the path planner to start.");
    ac_.waitForServer();
    ROS_INFO("Path planner started.");
  }

  ////////////////
  // Destructor //
  ////////////////

  ~PathPlannerClient() {}

  ////////////////////
  // Public Methods //
  ////////////////////

  bool compute_path(usv_sim::PathPlannerGoal goal)
  {
    ac_.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_INFO("Path planning finished: %s", state.toString().c_str());

      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        result_ = ac_.getResult();
        return true;
      }
      else
        return false; // fail to generate a path
    }
    else
    {
      ROS_ERROR("Path planning did not finish within 10 seconds.");
      return false;
    }
  }
};

#endif // PATH_PLANNER_CLIENT_H
