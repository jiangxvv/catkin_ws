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


#include"tug_controller_interface.h"

TugControllerInterface::TugControllerInterface(const VesselControlProperty &vessel, 
                         const double &vessel_length, const double &time_step, 
                         const double &kp, const double &k)
                        :vessel_(vessel), vessel_length_(vessel_length),
                        time_step_(time_step), kp_(kp), k_(k)
{
    name_=vessel_.name;
    xg_=vessel_.xg;
    
    timer_=nh_.createTimer(ros::Duration(time_step_),
                           &TugControllerInterface::timerCallback, this);
    
    Ptr_controller_=std::unique_ptr<FeedbackLinearController>(new                               FeedbackLinearController(vessel_, time_step_, kp_, k_));
    
    pubPtr_control_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Vector3>((name_+"/control"), 1000)));
    
    sub_touch_point_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe((name_+"/touch_point"), 1000,
                      &TugControllerInterface::callback_course, this)));
    
    sub_tug_motion_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe((name_+"/motion_state"), 1000,
                      &TugControllerInterface::callback_motionstate, this)));
    
}


void TugControllerInterface::timerCallback(const ros::TimerEvent &)
{
    Ptr_controller_->setVesselPosition(position_);
    Ptr_controller_->setVesselVelocity(velocity_);
    Ptr_controller_->setVesselAcceleration(acceleration_);
    
    Ptr_controller_->setCoursePosition(pd_);
    Ptr_controller_->setCourseVelocity(pd1_);
    Ptr_controller_->setCourseAcceleration(pd2_);
    
    actuation_=Ptr_controller_->ComputeControl();
    msg_actuation_.x=actuation_[0];
    msg_actuation_.y=actuation_[1];
    msg_actuation_.z=actuation_[2];
    
    pubPtr_control_->publish(msg_actuation_);
}


void TugControllerInterface::callback_course(const geometry_msgs::Point &msg)
{
    // cable's touch_point on the tug (aft point on the tug)
    double xt=msg.x;
    double yt=msg.y;
    
    // yaw angle of tug on the current time
    double fai=position_[2];
    double l=vessel_length_;
    
    // tug's desired position 
    xd_=xt+(l/2+xg_)*cos(fai);
    yd_=yt+(l/2+xg_)*sin(fai);
    
    // define the course
    pd_<<xd_, yd_, 0;
    pd_[2]=position_[2]; // ensure the yaw angle, need to be figured out
    
    pd1_<<0, 0, 0; // slowly to zero 
    pd2_<<0, 0, 0;
}


void TugControllerInterface::callback_motionstate(const gnc_msgs::MotionStamped &msg)
{
    
    current_time_=msg.time;
    position_<<msg.pose.x, msg.pose.y, msg.pose.z;
    velocity_<<msg.vel.x, msg.vel.y, msg.vel.z;
    acceleration_<<msg.accel.x, msg.accel.y, msg.accel.z;
}
