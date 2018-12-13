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

#ifndef BASIC_COURSE_H
#define BASIC_COURSE_H

#include<Eigen/Dense>
#include"basic_course.h"

class BasicCourse 
{
public:
    BasicCourse(const double& time_step):time_step_(time_step){}
    // course 
    Eigen::Vector3d pd_;
    Eigen::Vector3d pd1_;
    Eigen::Vector3d pd2_;
    
protected:
    double time_step_;
    
    double current_time_=0;
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d acceleration_;
    
    
public:
    
    
    void setPd(const Eigen::Vector3d& pd)
    {
        pd_=pd;
    }
    
    void setPd1(const Eigen::Vector3d& pd1)
    {
        pd1_=pd1;
    }
    
    void setPd2(const Eigen::Vector3d& pd2)
    {
        pd2_=pd2;
    }
    
    
    
    Eigen::Vector3d getPd()
    {
        return pd_;
    }
    
    Eigen::Vector3d getPd1()
    {
        return pd1_;
    }
    
    Eigen::Vector3d getPd2()
    {
        return pd2_;
    }
    
};



#endif