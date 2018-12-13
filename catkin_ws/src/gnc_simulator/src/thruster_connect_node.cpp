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


#include<ros/ros.h>
#include<memory>
#include"thruster_connect.h"

ThrusterConnect::ThrusterConnect(const double& xg, 
                                 const double& tug_length,
                                 const double& time_step,
                                 const double& cable_length,
                                 const Eigen::Vector3d& touch_point1,
                                 const Eigen::Vector3d& touch_point2)
        :xg_(xg), tug_length_(tug_length), 
         time_step_(time_step), cable_length_(cable_length),
         touch_point1_(touch_point1), touch_point2_(touch_point2)
         
{
    timer_=nh_.createTimer(ros::Duration(time_step_),
                           &ThrusterConnect::timerCallback, this);
    
    Ptr_cable_=std::unique_ptr<Cable>(new Cable(cable_length_));
    
    pubPtr_control_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Vector3>("mothership/control", 1000)));
    
    subPtr_tug1_motion_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("tug1/motion_state", 1000,
                      &ThrusterConnect::callback_tug1, this)));
    
    subPtr_tug2_motion_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("tug2/motion_state", 1000,
                      &ThrusterConnect::callback_tug2, this)));
    
    subPtr_mothership_motion_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("mothership/motion_state", 1000,
                      &ThrusterConnect::callback_mothership, this)));
}

void ThrusterConnect::timerCallback(const ros::TimerEvent&)
{
    cable1_force_=Ptr_cable_->conputeForce(cable1_length_);
    cable2_force_=Ptr_cable_->conputeForce(cable2_length_);
    
    // cable angle on the global coordinate
    double theta1=atan2((cable1_end_point_[1]-cable1_start_point_[1]),
                        (cable1_end_point_[0]-cable1_start_point_[0]));
    double theta2=atan2((cable2_end_point_[1]-cable2_start_point_[1]),
                        (cable2_end_point_[0]-cable2_start_point_[0]));
    
    // cable angle on the body coordinate
    cable1_angle_=theta1-fai1_;
    cable2_angle_=theta2-fai2_;
    
    double fx1=cable1_force_*cos(cable1_angle_);
    double fy1=cable1_force_*sin(cable1_angle_);
    double fx2=cable2_force_*cos(cable2_angle_);
    double fy2=cable2_force_*sin(cable2_angle_);
    
    double m=-fx1*touch_point1_[1]+fy1*touch_point1_[0]
             -fx2*touch_point2_[1]+fy1*touch_point2_[0];
             
    msg_force_.x=fx1+fx2;
    msg_force_.y=fy1+fy2;
    msg_force_.z=m;
    
    pubPtr_control_->publish(msg_force_);
}



void ThrusterConnect::callback_mothership(const gnc_msgs::MotionStamped& msg)
{
    mothership_position_<< msg.pose.x, msg.pose.y, msg.pose.z;
    //mothership_velocity_<<msg.vel.x, msg.vel.y, msg.vel.z;
    
}

void ThrusterConnect::callback_tug1(const gnc_msgs::MotionStamped& msg)
{
    tug1_position_<<msg.pose.x, msg.pose.y, msg.pose.z;
    double l=sqrt(pow(touch_point1_[0],2)+pow(touch_point1_[1], 2));
    fai_=mothership_position_[2];// yaw angle
    double theta=atan2(touch_point1_[1], touch_point1_[0]);// angle related with o
    cable1_start_point_[0]=mothership_position_[0]+l*cos(theta+fai_);
    cable1_start_point_[1]=mothership_position_[1]+l*sin(theta+fai_);
    
    fai1_=tug1_position_[2];
    cable1_end_point_[0]=tug1_position_[0]-(tug_length_/2.0+xg_)*cos(fai1_);
    cable1_end_point_[1]=tug1_position_[1]-(tug_length_/2.0+xg_)*sin(fai1_);
    
    cable1_length_=sqrt(pow((cable1_end_point_[0]-cable1_start_point_[0]),2)+
                        pow((cable1_end_point_[1]-cable1_start_point_[1]),2));
}

void ThrusterConnect::callback_tug2(const gnc_msgs::MotionStamped& msg)
{
    tug2_position_<<msg.pose.x, msg.pose.y, msg.pose.z;
    
    double l=sqrt(pow(touch_point2_[0],2)+pow(touch_point2_[1], 2));
    fai_=mothership_position_[2];// yaw angle
    double theta=atan2(touch_point2_[1], touch_point2_[0]);// angle related with o
    cable2_start_point_[0]=mothership_position_[0]+l*cos(theta+fai_);
    cable2_start_point_[1]=mothership_position_[1]+l*sin(theta+fai_);
    
    double fai2_=tug2_position_[2];
    cable2_end_point_[0]=tug2_position_[0]-(tug_length_/2.0+xg_)*cos(fai2_);
    cable2_end_point_[1]=tug2_position_[1]-(tug_length_/2.0+xg_)*sin(fai2_);
    
    cable2_length_=sqrt(pow((cable2_end_point_[0]-cable2_start_point_[0]),2)+
                        pow((cable2_end_point_[1]-cable2_start_point_[1]),2));
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "thruster_connect");
    
    double xg=-0.2;
    double tug_length=31.942;
    double time_step=0.05;
    double cable_length=950;
    Eigen::Vector3d touch_point1, touch_point2;
    touch_point1<<tug_length/2, 32.5, 0;
    touch_point2<<tug_length/2, 32.5, 0;
    
    ThrusterConnect thruster_connect(xg, tug_length, time_step, cable_length,
                                     touch_point1, touch_point2);
    
    thruster_connect.run();
    
    
}
    
    
    







