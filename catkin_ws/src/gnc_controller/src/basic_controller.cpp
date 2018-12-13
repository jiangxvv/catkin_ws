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

// basic class of controller

#include "basic_controller.h"
#include<cmath>

BasicController::BasicController(const VesselControlProperty& vessel, const double& step_size) : step_size_(step_size)
{
    mass_ = vessel.mass;
    radius_=vessel.radius;
    Irr_ = mass_*radius_*radius_;
    length_=vessel.length;
    a11_ = vessel.a11;
    a22_ = vessel.a22;
    a23_ = vessel.a23;
    a33_ = vessel.a33;
    xg_ = vessel.xg;
    d11_ = vessel.d11;
    d22_ = vessel.d22;
    d33_ = vessel.d33;
}

// rotation matrix definition
Eigen::Matrix3d BasicController::RotationMatrix(const Eigen::Vector3d& pose) const
{
    Eigen::Matrix3d rz;
    rz << cos(pose[2]), -sin(pose[2]), 0, sin(pose[2]), cos(pose[2]), 0, 0, 0, 1;
    return rz;
}

// inertia matrix of the vehicle
Eigen::Matrix3d BasicController::InertiaMatrix() const
{
    Eigen::Matrix3d inertia_mat;
    inertia_mat << mass_ + a11_, 0, 0, 0, mass_ + a22_, a23_, 0, a23_,
    Irr_ + a33_;
    return inertia_mat;
}

// inertial centripetal and coriolis matrix
Eigen::Matrix3d BasicController::InertiaCCMatrix(const Eigen::Vector3d& vec) const
{
    Eigen::Matrix3d inertia_cc_mat;
    inertia_cc_mat << 0, -mass_* vec[2], -a23_* vec[2] - a22_* vec[1],
    mass_* vec[2], 0, a11_* vec[0], a23_* vec[2] + a22_* vec[1],
    -a11_* vec[0], 0;
    return inertia_cc_mat;
}


// damping matrix of the vehicle
Eigen::Vector3d BasicController::DampingVector(const Eigen::Vector3d& vec) const
{
    double U =
          std::sqrt(std::pow(vec[0], 2) + std::pow(vec[1], 2)); // vehicle speed
    double d11=d11_*rou_/2*pow(length_,2)*U;
    double d22=d22_*rou_/2*pow(length_,2)*U;
    double d33=d33_*rou_/2*pow(length_,4)*U;
    Eigen::Vector3d damping1, damping2;
    damping1 << d11* vec[0], d22* vec[1] ,  d33* vec[2];

  // Xu|u| is different in forward and reverse conditions
  //double Xuu = (vec[0] > 0) ? 112.4 : 198.1;
  //damping2 << Xuu* vec[0] * std::fabs(vec[0]), 0, 0;
    return damping1;
}







