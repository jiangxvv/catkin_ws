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

#include "waypoint_tracking_client.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include "usv_sim/WaypointTrackingAction.h"

void WaypointTrackingClient::send_goal(Maneuver* mission)
{
  // define the waypoints
  usv_sim::WaypointTrackingGoal goal;

  goal.mission_type = mission->get_type();
  goal.mission_duration = mission->get_duration();

  std::vector<double> xcoor = mission->get_xcoor();
  std::vector<double> ycoor = mission->get_ycoor();
  std::vector<double> heading = mission->get_heading();

  if (mission->get_progress() <
      mission->get_size()) // no longer used since the lates update
  {
    for (auto p = xcoor.begin() + mission->get_progress(); p != xcoor.end();
         ++p)
    {
      goal.pos_x.push_back(*p);
    }

    for (auto p = ycoor.begin() + mission->get_progress(); p != ycoor.end();
         ++p)
    {
      goal.pos_y.push_back(*p);
    }

    for (auto p = heading.begin() + mission->get_progress(); p != heading.end();
         ++p)
    {
      goal.heading.push_back(*p);
    }
  }
  //    else
  //    {
  //        ROS_ERROR("%s is already completed.", mission->get_name().c_str());
  //        ROS_ERROR("Reset its progress before running it again.");
  //        return;
  //    }

  if (goal.pos_x.size() != goal.pos_y.size())
  {
    ROS_ERROR_STREAM(
        "The sizes of the coordinates don't match. No waypoints sent.");
    return;
  }

  if (goal.pos_x.empty() || goal.pos_y.empty())
  {
    ROS_ERROR_STREAM("The coordinates are empty. No waypoints sent");
    return;
  }

  if (ptr_mission_ != nullptr)
  {
    ROS_WARN("------ Previous mission ended (%u/%lu) ------",
             ptr_mission_->get_progress(), ptr_mission_->get_size() - 1);
  }

  // link the pointer to the current mission
  ptr_mission_ = mission;

  // reset the status variables
  init_progress_ = ptr_mission_->get_progress();
  mission_id_ = ptr_mission_->get_name();
  is_done_ = false;

  // display the information of the mission
  ROS_INFO("----- New maneuver sent to the server -----");
  ROS_INFO("Maneuver name: %s", ptr_mission_->get_name().c_str());
  //    ROS_INFO("Initial progress: %u/%lu", init_progress_,
  //    ptr_mission_->get_size()-1);
  ROS_INFO("Number of sent waypoints: %lu", goal.pos_x.size() - 1);
  ROS_DEBUG("The coordinates of the waypoints are:");
  ROS_DEBUG("wpt ----- x (m) ------- y (m)");

  for (uint32_t i = 0; i < goal.pos_x.size(); ++i)
  {
    if (i > 0)
    {
      ROS_DEBUG("%2u ------ %5.2f ------- %5.2f",
               i + ptr_mission_->get_progress(), goal.pos_x[i], goal.pos_y[i]);
    }
    else
    {
      ROS_DEBUG("%2u ------ %5.2f ------- %5.2f", i, goal.pos_x[i],
               goal.pos_y[i]);
    }
  }

  // send the goal to the action server
  ac_.sendGoal(goal, boost::bind(&WaypointTrackingClient::doneCb, this, _1, _2),
               boost::bind(&WaypointTrackingClient::activeCb, this),
               boost::bind(&WaypointTrackingClient::feedbackCb, this, _1));
}
