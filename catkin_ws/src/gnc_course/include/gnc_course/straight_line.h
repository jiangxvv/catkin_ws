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

#ifndef STRAIGHT_LINE_H
#define STRAIGHT_LINE_H

#include<Eigen/Dense>
#include"basic_course.h"

class StraightLine: public BasicCourse
{
public:
    StraightLine ( const double &time_step, 
                   const Eigen::Vector3d& accel, 
                   const Eigen::Vector3d& init_vel, 
                   const Eigen::Vector3d& init_pose)
    :BasicCourse(time_step), accel_(accel), init_vel_(init_vel), init_pose_(init_pose)
    {}
    
    
    
    
    
    void computeStraightLine()
    {
        double t=current_time_;
        pd2_=accel_;
        pd1_=init_vel_+t*accel_;
        pd_<<init_pose_+init_vel_*t+accel_*pow(t, 2)/2; 
        current_time_=current_time_+time_step_;
    }
    
    
private:
    Eigen::Vector3d accel_;
    Eigen::Vector3d init_vel_;
    Eigen::Vector3d init_pose_;
};


#endif