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

// the controller interface of mothership
#include<ros/ros.h>
#include<cmath>
#include"central_controller_interface.h"
#include"gnc_msgs/MotionStamped.h"


CentralControllerInterface::CentralControllerInterface(
    const VesselControlProperty& vessel,
    const double& time_step,const double& kp, 
    const double& k,const double& cable_length,
    const Eigen::Vector3d& touch_position1,
    const Eigen::Vector3d& touch_position2)
    :vessel_(vessel), time_step_(time_step), kp_(kp), k_(k),
    cable_length_(cable_length), touch_position1_(touch_position1),
    touch_position2_(touch_position2)
{
    timer_=nh_.createTimer(ros::Duration(time_step_),
                           &CentralControllerInterface::timerCallback, this);
    
    Ptr_controller_=std::unique_ptr<FeedbackLinearController>(new FeedbackLinearController(vessel_, time_step_, kp_, k_));
    
    Ptr_allocation_=std::unique_ptr<VariableThrusterAllocation>(new
        VariableThrusterAllocation(touch_position1_, touch_position2_));
    
    Ptr_cable_=std::unique_ptr<Cable>(new Cable(cable_length_));
    
    pubPtr_pd1_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Point>("tug1/touch_point", 1000)));
    
    pubPtr_pd2_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Point>("tug2/touch_point", 1000)));
    
        
    sub_course_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("mothership/course", 1000,
            &CentralControllerInterface::callback_course, this)));
        
    sub_motion_state_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("mothership/motion_state", 1000,
        &CentralControllerInterface::callback_motionstate, this)));
    
}





// private method

void CentralControllerInterface::timerCallback(const ros::TimerEvent&)
{
    actuation_=Ptr_controller_->ComputeControl()*1E3*9.81;
    tug_force_=Ptr_allocation_->ComputeTug2Force();//error here 
    std::cout<<"actuation_= "<<actuation_<<std::endl;
    std::cout<<"tug_force_= "<<tug_force_<<std::endl;
    f1_=tug_force_[0];
    f2_=tug_force_[2];
    alpha1_=tug_force_[1]; 
    alpha2_=tug_force_[3];
    
    double length1=Ptr_cable_->computeLength(f1_);
    double length2=Ptr_cable_->computeLength(f2_);
    
    // transfer the position in body coordinate to the global coordinate
    // compute the disire position of two tugs in the global coordinate
    double fai=pose_[2];
    Eigen::Matrix2d Trans;
    Trans<<cos(fai), -sin(fai), sin(fai), cos(fai);
    std::cout<<"Trans=  "<<Trans<<std::endl;
    std::cout<<"length1=  "<<length1<<std::endl;
    std::cout<<"touch_position1_=  "<<touch_position1_<<std::endl;
    
    
    // tug position in body coordinate
    double x_temp1=touch_position1_[0]+length1*cos(alpha1_);
    double y_temp1=touch_position1_[1]+length1*sin(alpha1_);
    double x_temp2=touch_position2_[0]+length1*cos(alpha2_);
    double y_temp2=touch_position2_[1]+length1*sin(alpha2_);
    
    std::cout<<"x_temp1=  "<< y_temp1<<std::endl;
    std::cout<<"x_temp2=  "<< y_temp2<<std::endl;
    Eigen::Vector2d pose0;
    Eigen::Vector2d pose_temp1;
    Eigen::Vector2d pose_temp2;
    pose_temp1<<x_temp1, y_temp1;
    pose_temp2<<x_temp2, y_temp2;
    pose0<<pose_[0], pose_[1];
    
    
    position1_=Trans*pose_temp1+pose0;
    position2_=Trans*pose_temp2+pose0;
    
    std::cout<<"position1_=  :"<<position1_<<std::endl;
    std::cout<<"position2_=  :"<<position2_<<std::endl;
    
    msg_tug1_pose.x=position1_[0];
    msg_tug1_pose.y=position1_[1];
    msg_tug1_pose.z=position1_[2];
    
    msg_tug2_pose.x=position2_[0];
    msg_tug2_pose.y=position2_[1];
    msg_tug2_pose.z=position2_[2];

    pubPtr_pd1_->publish(msg_tug1_pose);
    pubPtr_pd2_->publish(msg_tug2_pose);
    
}


void CentralControllerInterface::callback_motionstate(
    const gnc_msgs::MotionStamped & msg)
{
    // receive the motion state msg
    current_time_=msg.time;
    pose_<<msg.pose.x, msg.pose.y, msg.pose.z;
    velocity_<<msg.vel.x, msg.vel.y, msg.vel.z;
    acceleration_<<msg.accel.x, msg.accel.y, msg.accel.z;
    
    // set the motion state in the controller by set function
    Ptr_controller_->setVesselPosition(pose_);
    Ptr_controller_->setVesselVelocity(velocity_);
    Ptr_controller_->setVesselAcceleration(acceleration_);
}




void CentralControllerInterface::callback_course(const gnc_msgs::course &msg_course)
{
    Eigen::Vector3d pose;
    pose<<msg_course.pose.x, msg_course.pose.y, msg_course.pose.z;
    Ptr_controller_->setCoursePosition(pose);
    
    Eigen::Vector3d vel;
    vel<<msg_course.velocity.x, msg_course.velocity.y, msg_course.velocity.z;
    Ptr_controller_->setCourseVelocity(vel);
    
    Eigen::Vector3d acc;
    acc<<
        msg_course.acceleration.x, msg_course.acceleration.y, msg_course.acceleration.z;
    Ptr_controller_->setCourseAcceleration(acc);
}













