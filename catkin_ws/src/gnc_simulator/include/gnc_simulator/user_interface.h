/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Xu Jiang <jiangxvv@sjtu.edu.cn>
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
 *   * Neither the name of Xu Jiang nor the names of its contributors may
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

#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include<Eigen/Dense>
#include<ros/ros.h>
#include<memory>
#include<visualization_msgs/Marker.h>
#include"gnc_msgs/MotionStamped.h"
#include<geometry_msgs/Pose2D.h>
#include <gnc_msgs/MotionStamped.h>


namespace ros
{
    class NodeHandle;
    class Subscriber;
    class Publisher;
}
    
namespace tf
{
    class TransformBroadcaster;
}
        


class UserInterface 
{
public:
    UserInterface();
    
    int run();
    
    ~UserInterface();
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<ros::Subscriber> sub_mothership_pose_;
    std::unique_ptr<ros::Subscriber> sub_tug1_pose_;
    std::unique_ptr<ros::Subscriber> sub_tug2_pose_;
    
    std::unique_ptr<ros::Publisher> pub_mothership_;
    std::unique_ptr<ros::Publisher> pub_tug1_;
    std::unique_ptr<ros::Publisher> pub_tug2_;
    std::unique_ptr<ros::Publisher> pub_cable1_;
    std::unique_ptr<ros::Publisher> pub_cable2_;
    
    visualization_msgs::Marker mothership_;
    visualization_msgs::Marker tug1_;
    visualization_msgs::Marker tug2_;
    visualization_msgs::Marker cable1_;
    visualization_msgs::Marker cable2_;
    
    Eigen::Vector3d pose_motherhsip, pose_tug1, pose_tug2;
    
    // coordinate tranformation
    std::unique_ptr<tf::TransformBroadcaster> 
        br_ef_; // the earth-fixed frame broadcaster
        
    std::unique_ptr<tf::TransformBroadcaster>
        br_bf_; // the body-fixed frame broadcaster
    
    void init_marker();
    void callback_mothership(const gnc_msgs::MotionStamped& msg);
    void callback_tug1(const gnc_msgs::MotionStamped& msg);
    void callback_tug2(const gnc_msgs::MotionStamped& msg);
    
//     void initial_vessel_marker(const 
    
    
};


#endif