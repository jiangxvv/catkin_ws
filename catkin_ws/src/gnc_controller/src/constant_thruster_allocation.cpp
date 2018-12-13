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

#include<Eigen/Dense>
#include"constant_thruster_allocation.h"

ConstantThrusterAllocation::ConstantThrusterAllocation(const Eigen::Vector3d tug1_position, const Eigen::Vector3d tug2_position)
    :tug1_position_(tug1_position), tug2_position_(tug2_position)
{
    
}

ConstantThrusterAllocation::ConstantThrusterAllocation(const Eigen::Vector3d tug1_position, const Eigen::Vector3d tug2_position, const Eigen::Vector3d tug3_position, const Eigen::Vector3d tug4_position)
    :tug1_position_(tug1_position), tug2_position_(tug2_position),
     tug3_position_(tug3_position), tug4_position_(tug4_position)
{

}

void ConstantThrusterAllocation::ConfigureMatrix2()
{
    x1_=tug1_position_[0]; x2_=tug2_position_[0]; 
    y1_=tug1_position_[1]; y2_=tug2_position_[1]; 
    alpha1_=tug1_position_[2]; alpha2_=tug2_position_[2]; 
    alpha1_=alpha1_/180.0*3.14;
    alpha2_=alpha2_/180.0*3.14;
    
    B2_<<cos(alpha1_), sin(alpha1_), 
        -y1_*cos(alpha1_)+x1_*sin(alpha1_),
        cos(alpha2_), sin(alpha2_), 
        -y2_*cos(alpha2_)+x2_*sin(alpha2_);
}


void ConstantThrusterAllocation::ConfigureMatrix4()
{
    x1_=tug1_position_[0]; x2_=tug2_position_[0]; 
    x3_=tug3_position_[0]; x4_=tug4_position_[0];
    y1_=tug1_position_[1]; y2_=tug2_position_[1]; 
    y3_=tug3_position_[1]; y4_=tug4_position_[1];
    alpha1_=tug1_position_[2]; alpha2_=tug2_position_[2]; 
    alpha3_=tug3_position_[2]; alpha4_=tug4_position_[2];
    
    alpha1_=alpha1_/180.0*3.14;
    alpha2_=alpha2_/180.0*3.14;
    alpha3_=alpha3_/180.0*3.14;
    alpha4_=alpha4_/180.0*3.14;
    
    B4_<<cos(alpha1_), sin(alpha1_), 
        -y1_*cos(alpha1_)+x1_*sin(alpha1_),
        cos(alpha2_), sin(alpha2_), 
        -y2_*cos(alpha2_)+x2_*sin(alpha2_),
        cos(alpha3_), sin(alpha3_), 
        -y3_*cos(alpha3_)+x3_*sin(alpha3_),
        cos(alpha4_), sin(alpha4_),
        -y4_*cos(alpha4_)+x4_*sin(alpha4_);
}



ConstantThrusterAllocation::Vector2d ConstantThrusterAllocation::ComputeTug2Force()
{
    // problem here
    tug2_force_=B2_.colPivHouseholderQr().solve(force_net_);
}


ConstantThrusterAllocation::Vector4d ConstantThrusterAllocation::ComputeTug4Force()
{
    // problem here
    tug4_force_=B4_.colPivHouseholderQr().solve(force_net_);
}









