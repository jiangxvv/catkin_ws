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

#include "viz_trajectory.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/Pose2D.h>

using namespace std;

/////////////////
// Constructor //
/////////////////

VizTrajectory::VizTrajectory()
{
  // populate node handle
  nh_ = unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

  // define the subscriber/publisher
  sub_pos_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(
      nh_->subscribe("ship/pose", 1000, &VizTrajectory::sub_callback, this)));
  pub_traj_ = unique_ptr<ros::Publisher>(
      new ros::Publisher(nh_->advertise<visualization_msgs::Marker>(
          "ship/trajectory_marker", 1000)));
  pub_ship_ = unique_ptr<ros::Publisher>(new ros::Publisher(
      nh_->advertise<visualization_msgs::Marker>("ship/ship_marker", 10)));

  // initialize the transformations
  br_ef_ = unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
  br_bf_ = unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);

  // initialize the trajectory marker
  trajectory_.header.frame_id = "world";
  trajectory_.header.stamp = ros::Time::now();
  trajectory_.ns = "ship/viz";
  trajectory_.action = visualization_msgs::Marker::ADD;
  trajectory_.pose.orientation.w = 1.0;
  trajectory_.id = 0;
  trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_.scale.x = 0.05; // line width
  trajectory_.color.r = 1.0;
  trajectory_.color.g = 0.0;
  trajectory_.color.b = 1.0;
  trajectory_.color.a = 1.0;

  // initialize the ship marker
  ship_.header.frame_id = "world";
  ship_.header.stamp = ros::Time::now();
  ship_.ns = "ship/viz";
  ship_.action = visualization_msgs::Marker::ADD;
  ship_.pose.orientation.w = 1.0;
  ship_.id = 0;
  ship_.type = visualization_msgs::Marker::TRIANGLE_LIST;
  ship_.scale.x = 1;
  ship_.scale.y = 1;
  ship_.scale.z = 1;
  ship_.color.r = 1.0;
  ship_.color.g = 1.0;
  ship_.color.b = 0.0;
  ship_.color.a = 0.3;
}

////////////////
// Destructor //
////////////////

VizTrajectory::~VizTrajectory() {}

////////////////////
// Public Methods //
////////////////////

int VizTrajectory::run()
{
  // control the publishing rate
  ros::Rate r(10);

  while (ros::ok())
  {
    pub_traj_->publish(trajectory_);
    pub_ship_->publish(ship_);

    ros::spinOnce();
    r.sleep();
  }

  return EXIT_FAILURE;
}

/////////////////////
// Private methods //
/////////////////////

void VizTrajectory::sub_callback(const geometry_msgs::Pose2DConstPtr& msg)
{
  geometry_msgs::Point p;

  // update the points of the line_strip
  p.x = msg->x;
  p.y = -msg->y; // add the minus sign because of the transformation from world
                 // to the earth frame
  p.z = 0;

  // limit the length of trajectory
  if (trajectory_.points.size() > 2000)
  {
    trajectory_.points.erase(trajectory_.points.begin());
  }

  // update the trajectory
  trajectory_.points.push_back(p);

  // update the ship marker
  geometry_msgs::Point p1, p2;
  p.x = msg->x + 8 * cos(msg->theta);
  p.y = -(msg->y + 8 * sin(msg->theta));
  p1.x = msg->x - 3 * cos(msg->theta) - 5 * sin(msg->theta);
  p1.y = -(msg->y - 3 * sin(msg->theta) + 5 * cos(msg->theta));
  p2.x = msg->x - 3 * cos(msg->theta) + 5 * sin(msg->theta);
  p2.y = -(msg->y - 3 * sin(msg->theta) - 5 * cos(msg->theta));

  if (ship_.points.empty())
  {
    ship_.points.push_back(p);
    ship_.points.push_back(p1);
    ship_.points.push_back(p2);
  }
  else
  {
    ship_.points[0] = p;
    ship_.points[1] = p1;
    ship_.points[2] = p2;
  }

  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0, 0, 0));
  q.setRPY(M_PI, 0, 0);
  transform.setRotation(q);
  br_ef_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                             "world", "Earth-fixed frame"));

  transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br_bf_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                             "Earth-fixed frame", "Ship"));
}
