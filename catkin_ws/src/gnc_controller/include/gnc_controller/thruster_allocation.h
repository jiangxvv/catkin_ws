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

#include <ros/ros.h>
#include<Eigen/Dense>
#include<cmath>
#include<boost/math/constants/constants.hpp>
#include<memory>
#include<cstdlib>

#include"usv_towing/actuation.h"
#include"usv_towing/control.h"



class ThrusterAllocation
{

	// const pi
	const double pi=boost::math::constants::pi<double>();

public:
	// constructor
	ThrusterAllocation(const double& time_step);

	// public method
	int run()
	{
		ros::spin();
		return EXIT_FAILURE;
	}

 	~ThrusterAllocation();

private:
// private field

	// mew matrix types
	typedef Eigen::Matrix<double, 4, 1> Vector4d;
	typedef Eigen::Matrix<double, 3, 4> Matrix34d;

	// configure matrix
	ThrusterAllocation::Matrix34d B_;

	// T_net, T_i
	ThrusterAllocation::Vector4d  Tau_i_;

	Eigen::Vector3d Tau_net_;

	

	// positions and force orientations of four tugs
	double x1_, y1_, alpha1_;
	double x2_, y2_ ,alpha2_;
	double x3_, y3_, alpha3_;
	double x4_, y4_, alpha4_;

	// ros nodehandle
	ros::NodeHandle nh_;

	// timer which controls simulation and the publishing rate
	ros::Timer timer_;

	// publishers and messages
	std::unique_ptr<ros::Publisher> pubPtr_act_;
	usv_towing::actuation msg_act_;

	// subscriber
	std::unique_ptr<ros::Subscriber> subPtr_ctrl_;

	// time step of message publishing
	double step_size_=0.1;

	// flag
	bool received_control_;

// private methods
	void get_configure_params();
	//ThrusterAllocation::Vector4d ComputeOut();
	void timerCallback(const ros::TimerEvent&);
	void callback_sub(const usv_towing::control& msg_ctrl);
};