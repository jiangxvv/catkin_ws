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

#ifndef WAYPOINT_TRACKING_MISSION_H
#define WAYPOINT_TRACKING_MISSION_H

#include <ros/ros.h>
#include <vector>

class Maneuver
{
  friend class UserInterface;

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // target position
  double goal_xcoor_;
  double goal_ycoor_;
  double goal_psi_;

  // waypoints
  std::vector<double> wpt_xcoor_;
  std::vector<double> wpt_ycoor_;
  std::vector<double> wpt_heading_;

  // ID
  std::string id_;

  // mission progress
  uint32_t progress_ = 0;

  // mission type
  // waypoint tracking -- falsoe
  // station keeping   -- true
  bool station_keeping_on_;

  // duration of the station-keeping task
  ros::Duration duration_;

  // mission description
  std::string description_;

public:
  /////////////////
  // Constructor //
  /////////////////

  Maneuver() = default;
  Maneuver(const std::vector<double>& wpt_xcoor,
           const std::vector<double>& wpt_ycoor,
           const std::vector<double>& wpt_heading,
           const std::string& mission_name, const bool& mission_type,
           const double& duration)
      : wpt_xcoor_(wpt_xcoor), wpt_ycoor_(wpt_ycoor), wpt_heading_(wpt_heading),
        id_(mission_name), station_keeping_on_(mission_type),
        duration_(duration)
  {
  }

  ////////////////////
  // Public Methods //
  ////////////////////

  // member function resetting the progress
  void reset_progress()
  {
    progress_ = 0;
    ROS_INFO("The progress of maneuver %s is reset.", id_.c_str());
  }

  void set_progress(uint32_t p)
  {
    if (p > wpt_xcoor_.size())
    {
      ROS_ERROR("set_progress failed: the progress shouldn't be larger than "
                "the waypoint number.");
      return;
    }
    progress_ = p;
  }

  std::string get_name() const { return id_; }
  uint32_t get_progress() const { return progress_; }
  size_t get_size() const { return wpt_xcoor_.size(); }
  std::vector<double> get_xcoor() const { return wpt_xcoor_; }
  std::vector<double> get_ycoor() const { return wpt_ycoor_; }
  std::vector<double> get_heading() const { return wpt_heading_; }
  bool get_type() const { return station_keeping_on_; }
  bool empty() const { return wpt_xcoor_.empty(); }
  bool is_station_keeping() const { return station_keeping_on_; }
  double get_duration() const { return duration_.toSec(); }
  std::string get_description() const { return description_; }
};

#endif // WAYPOINT_TRACKING_MISSION_H
