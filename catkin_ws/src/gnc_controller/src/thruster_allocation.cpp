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

// class for thruster allocation to each tug 

#include "thruster_allocation.h"

#include <ros/ros.h>
#include<Eigen/Dense>
#include<cmath>
#include<boost/math/constants/constants.hpp>
#include<memory>
#include<cstdlib>
#include"usv_towing/actuation.h"
#include"usv_towing/control.h"

// constructor
ThrusterAllocation::ThrusterAllocation(const double& time_step):step_size_(time_step)
{
    // get the configure params(tug position and force orientation)
    get_configure_params();

    // define the timer
    timer_=nh_.createTimer(ros::Duration(step_size_),
        &ThrusterAllocation::timerCallback,this);
    
    // define the publishers
    pubPtr_act_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<usv_towing::actuation>("tug/actuation",1000)));
    
    // define the subscriber
    subPtr_ctrl_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("CentralController/control", 1000, &ThrusterAllocation::callback_sub, this)));
    
    
}

// destructor
ThrusterAllocation::~ThrusterAllocation(){}

// private methods

void ThrusterAllocation::get_configure_params()
{
    // define parameters of configure matrix
    const std::string param_x1="/configure_params/position_and_angle/tug1_x";
    const std::string param_y1="/configure_params/position_and_angle/tug1_y";
    const std::string param_alpha1="/configure_params/position_and_angle/tug1_alpha";

    const std::string param_x2="/configure_params/position_and_angle/tug2_x";
    const std::string param_y2="/configure_params/position_and_angle/tug2_y";
    const std::string param_alpha2="/configure_params/position_and_angle/tug2_alpha";

    const std::string param_x3="/configure_params/position_and_angle/tug3_x";
    const std::string param_y3="/configure_params/position_and_angle/tug3_y";
    const std::string param_alpha3="/configure_params/position_and_angle/tug3_alpha";

    const std::string param_x4="/configure_params/position_and_angle/tug4_x";
    const std::string param_y4="/configure_params/position_and_angle/tug4_y";
    const std::string param_alpha4="/configure_params/position_and_angle/tug4_alpha";

    ros::param::get(param_x1,x1_);
    ros::param::get(param_y1,y1_);
    ros::param::get(param_alpha1,alpha1_);

    ros::param::get(param_x2,x2_);
    ros::param::get(param_y2,y2_);
    ros::param::get(param_alpha2,alpha2_);

    ros::param::get(param_x3,x3_);
    ros::param::get(param_y3,y3_);
    ros::param::get(param_alpha3,alpha3_);

    ros::param::get(param_x4,x4_);
    ros::param::get(param_y4,y4_);
    ros::param::get(param_alpha4,alpha4_);
    
    alpha1_=alpha1_/180.0*3.14;
    alpha2_=alpha2_/180.0*3.14;
    alpha3_=alpha3_/180.0*3.14;
    alpha4_=alpha4_/180.0*3.14;

    B_<<cos(alpha1_), sin(alpha1_), 
        -y1_*cos(alpha1_)+x1_*sin(alpha1_),
        cos(alpha2_), sin(alpha2_), 
        -y2_*cos(alpha2_)+x2_*sin(alpha2_),
        cos(alpha3_), sin(alpha3_), 
        -y3_*cos(alpha3_)+x3_*sin(alpha3_),
        cos(alpha4_), sin(alpha4_),
        -y4_*cos(alpha4_)+x4_*sin(alpha4_);
}


void ThrusterAllocation::callback_sub(const usv_towing::control& msg_ctrl)
{
    //received_control_=true；

    Tau_net_<<msg_ctrl.surge, msg_ctrl.sway, msg_ctrl.yaw;
}

void ThrusterAllocation::timerCallback(const ros::TimerEvent&)
{
    // solve the linear equations
    Tau_i_=B_.colPivHouseholderQr().solve(Tau_net_);

    // message
    msg_act_.tug1=Tau_i_(0);
    msg_act_.tug2=Tau_i_(1);
    msg_act_.tug3=Tau_i_(2);
    msg_act_.tug4=Tau_i_(3);

    pubPtr_act_->publish(msg_act_);

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"ThrusterAllocation");
    
    ThrusterAllocation ThrusterAllocation(0.1);

    return ThrusterAllocation.run();
}





