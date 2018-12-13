/*
 * 
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


#ifndef THRUSTER_CONNECT_H
#define THRUSTER_CONNECT_H

#include<ros/ros.h>
#include<gnc_msgs/MotionStamped.h>
#include<geometry_msgs/Vector3.h>

#include<Eigen/Dense>
#include<memory>
#include<gnc_cable/cable.h>



class ThrusterConnect 
{
public:
    ThrusterConnect(const double& xg, 
                    const double& tug_length,
                    const double& time_step,
                    const double& cable_length,
                    const Eigen::Vector3d& touch_point1,
                    const Eigen::Vector3d& touch_point2);
    
    int run()
    {
        ros::spin();
        return EXIT_FAILURE;
    }
    
private:
    
    ros::NodeHandle nh_;
    ros::Timer timer_;
    
    std::unique_ptr<ros::Publisher> pubPtr_control_;
    geometry_msgs::Vector3 msg_force_;
    
    std::unique_ptr<ros::Subscriber> subPtr_mothership_motion_;
    std::unique_ptr<ros::Subscriber> subPtr_tug1_motion_;
    std::unique_ptr<ros::Subscriber> subPtr_tug2_motion_;
    
    
    std::unique_ptr<Cable> Ptr_cable_;
    
    double xg_;
    double tug_length_;
    
    double time_step_;
    double cable_length_;
    
    Eigen::Vector3d touch_point1_; // cable1 touch point on the mothership
    Eigen::Vector3d touch_point2_; // cable2 touch point on the mothership
    
    Eigen::Vector2d cable1_start_point_;
    Eigen::Vector2d cable1_end_point_;
    double cable1_length_; // length of cable1 on the current time 
    double cable1_angle_, cable2_angle_; // cable angle on the body coordinate
    
    Eigen::Vector2d cable2_start_point_;
    Eigen::Vector2d cable2_end_point_;
    double cable2_length_;
    
    
    
    double cable1_force_;
    double cable2_force_;
    
    Eigen::Vector3d tug1_position_;
    Eigen::Vector3d tug2_position_;
    double fai1_, fai2_;
    
    Eigen::Vector3d mothership_position_;
    double fai_;
    
    
    void timerCallback(const ros::TimerEvent&);
    void callback_tug1(const gnc_msgs::MotionStamped& msg);
    void callback_tug2(const gnc_msgs::MotionStamped& msg);
    void callback_mothership(const gnc_msgs::MotionStamped& msg);
    
};







#endif