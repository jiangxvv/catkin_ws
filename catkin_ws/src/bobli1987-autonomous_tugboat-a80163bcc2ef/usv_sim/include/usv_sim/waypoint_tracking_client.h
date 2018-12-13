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

#ifndef WAYPOINT_TRACKING_CLIENT_H
#define WAYPOINT_TRACKING_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "usv_sim/WaypointTrackingAction.h"
#include <geometry_msgs/Pose2D.h>
#include "maneuver.h"

class WaypointTrackingClient
{
private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // simple action client
  actionlib::SimpleActionClient<usv_sim::WaypointTrackingAction> ac_;

  // pointer to the current mission
  Maneuver* ptr_mission_ = nullptr;

  // initial progress of the mission
  uint32_t init_progress_;

  // the id of the current mission
  std::string mission_id_;

  // whether the current mission is done
  bool is_done_ = false;

public:
  /////////////////
  // Constructor //
  /////////////////

  WaypointTrackingClient(const std::string& server_name)
      : ac_(server_name, true)
  {
    ROS_INFO("Waiting for the waypoint-tracking server to start.");
    ac_.waitForServer();
    ROS_INFO("Waypoint-tracking server started.");
  }

  ////////////////
  // Destructor //
  ////////////////

  ~WaypointTrackingClient() {}

  ////////////////////
  // Public Methods //
  ////////////////////

  // send goal to the action server
  void send_goal(Maneuver*);
  
  // cancel all goals running on the action server
  void cancel_goal() 
  { 
    ac_.cancelAllGoals();
    ROS_WARN("The running goal is canceled.");
  }

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const usv_sim::WaypointTrackingResultConstPtr& result)
  {
    is_done_ = true;
    ROS_INFO("Maneuver finished in state [%s]", state.toString().c_str());
    ROS_DEBUG("The last waypoint reached: wpt%i", result->final_wpt);
    ROS_INFO("------------------------------------------");
  }

  // Called once when the goal becomes active
  void activeCb() { ROS_INFO("The goal just went active."); }

  // Called every time feedback is received for the goal
  void feedbackCb(const usv_sim::WaypointTrackingFeedbackConstPtr& feedback)
  {
    ptr_mission_->set_progress(feedback->progress + init_progress_);
    ROS_DEBUG("Progress of the maneuver: %u/%lu", ptr_mission_->get_progress(),
             ptr_mission_->get_size() - 1);

    if (ptr_mission_->get_type() &&
        (ptr_mission_->get_progress() == ptr_mission_->get_size() - 1))
      ROS_INFO("The vehicle is now in station-keeping mode.");
  }

  // return whether the current mission is finished or not
  bool is_done() const { return is_done_; }

  // return the id of the current mission
  std::string mission_id() const { return mission_id_; }
};

#endif // WAYPOINT_TRACKING_CLIENT_H
