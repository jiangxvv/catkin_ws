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


#include"user_interface.h"
#include<ros/ros.h>
#include<Eigen/Dense>
#include<gnc_msgs/MotionStamped.h>
#include<tf/transform_broadcaster.h>
#include<cmath>
#include<cstdlib>
#include<geometry_msgs/Point.h>


UserInterface::UserInterface()
{
    // motion state subscriber
    sub_mothership_pose_=std::unique_ptr<ros::Subscriber>(new 
        ros::Subscriber(nh_.subscribe("mothership/state", 1000, 
            &UserInterface::callback_mothership, this)));
    sub_tug1_pose_=std::unique_ptr<ros::Subscriber>(new 
        ros::Subscriber(nh_.subscribe("tug1/state", 1000, 
            &UserInterface::callback_tug1, this)));
    sub_tug2_pose_=std::unique_ptr<ros::Subscriber>(new 
        ros::Subscriber(nh_.subscribe("tug2/state", 1000, 
            &UserInterface::callback_tug2, this)));
    
    //maker publisher
    pub_mothership_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<visualization_msgs::Marker>("motherhsip/marker", 10)));
    pub_tug1_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<visualization_msgs::Marker>("tug1/marker", 10)));
    pub_tug2_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<visualization_msgs::Marker>("tug2/marker", 10)));
    pub_cable1_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<visualization_msgs::Marker>("cable1/marker", 10)));
    pub_cable2_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<visualization_msgs::Marker>("cable2/marker", 10)));
    
    // initialize the transformation
    br_ef_=std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
    br_bf_=std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
    
    // initialize the vessel and cable maker
    mothership_.header.frame_id="world";
    mothership_.header.stamp=ros::Time::now();
    mothership_.ns="ship/viz";
    mothership_.action=visualization_msgs::Marker::ADD;
    mothership_.pose.orientation.w=1.0;
    mothership_.id=0;
    mothership_.type=visualization_msgs::Marker::TRIANGLE_LIST;
    mothership_.scale.x = 1;
    mothership_.scale.y = 1;
    mothership_.scale.z = 1;
    mothership_.color.r = 1.0;
    mothership_.color.g = 1.0;
    mothership_.color.b = 0.0;
    mothership_.color.a = 0.3;
//     mothership_
    
    
    
    // initialize the tug1 marker
    tug1_.header.frame_id = "world";
    tug1_.header.stamp = ros::Time::now();
    tug1_.ns = "ship/viz";
    tug1_.action = visualization_msgs::Marker::ADD;
    tug1_.pose.orientation.w = 1.0;
    tug1_.id = 0;
    tug1_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    tug1_.scale.x = 1;
    tug1_.scale.y = 1;
    tug1_.scale.z = 1;
    tug1_.color.r = 1.0;
    tug1_.color.g = 1.0;
    tug1_.color.b = 0.0;
    tug1_.color.a = 0.3;
    
    
    
    // initialize the tug1 marker
    tug2_.header.frame_id = "world";
    tug2_.header.stamp = ros::Time::now();
    tug2_.ns = "ship/viz";
    tug2_.action = visualization_msgs::Marker::ADD;
    tug2_.pose.orientation.w = 1.0;
    tug2_.id = 0;
    tug2_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    tug2_.scale.x = 1;
    tug2_.scale.y = 1;
    tug2_.scale.z = 1;
    tug2_.color.r = 1.0;
    tug2_.color.g = 1.0;
    tug2_.color.b = 0.0;
    tug2_.color.a = 0.3;
    
    
    cable1_.header.frame_id="world";
    cable1_.header.stamp=ros::Time::now();
    cable1_.ns = "ship/viz";
    cable1_.action = visualization_msgs::Marker::ADD;
    cable1_.pose.orientation.w = 1.0;
    cable1_.id = 0;
    cable1_.type = visualization_msgs::Marker::LINE_STRIP;
    cable1_.scale.x = 0.1;
    cable1_.color.r = 1.0;
    cable1_.color.g = 1.0;
    cable1_.color.b = 0.0;
    cable1_.color.a = 0.3;
    
    
    cable2_.header.frame_id="world";
    cable2_.header.stamp=ros::Time::now();
    cable2_.ns = "ship/viz";
    cable2_.action = visualization_msgs::Marker::ADD;
    cable2_.pose.orientation.w = 1.0;
    cable2_.id = 0;
    cable2_.type = visualization_msgs::Marker::LINE_STRIP;
    cable2_.scale.x = 0.1;
    cable2_.color.r = 1.0;
    cable2_.color.g = 1.0;
    cable2_.color.b = 0.0;
    cable2_.color.a = 0.3;
       
}





int UserInterface::run()
{
    ros::Rate r(10);
    
    while (ros::ok())
    {
        pub_cable1_->publish(cable1_);
        pub_cable2_->publish(cable2_);
        pub_mothership_->publish(mothership_);
        pub_tug1_->publish(tug1_);
        pub_tug2_->publish(tug2_);
        
        ros::spinOnce();
        r.sleep();
    }
    
    return EXIT_FAILURE;
}


void UserInterface::callback_mothership(const gnc_msgs::MotionStamped& msg)
{
    double t=msg.time;
    geometry_msgs::Point p;
    p.x=msg.pose.x; 
    p.y=-msg.pose.y; // add the minus sign to transfer from world to the earth frame
    p.z=0;
    
    
}











