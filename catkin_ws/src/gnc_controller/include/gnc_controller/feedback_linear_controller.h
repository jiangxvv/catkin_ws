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

// class for feedback linear controller
#ifndef FEEDBACK_LINEAR_CONTROLLER_H
#define FEEDBACK_LINEAR_CONTROLLER_H

#include<iostream>
#include<memory>
#include<Eigen/Dense>
#include<vector>
#include<string>
#include<cmath>

#include"basic_controller.h"

// class feedback linearization controller, which is derived from BasicController

class FeedbackLinearController: public BasicController
{
private:
    // controller gains 
    double kp_, k_;
    
public:
    
    // brief a constructor
    FeedbackLinearController(const VesselControlProperty& vessel, const double& step_size, const double& kp, const double& k);
   
    // brief a destructor
    ~FeedbackLinearController(){}
    
    // brief compute control output
    Eigen::Vector3d ComputeControl();
    
    
    

    // return gain  kp 
    double getKpGain() const
    {
        return kp_;
    }
    
    // return gain  k 
    double getKGain() const
    {
        return k_;
    }
    
    
    // set kp gain
    void setKpGain(const double& input)
    {
        kp_=input;
    }
    
    
    // set k gain
    void setKGain(const double& input)
    {
        k_=input;
    }
    
};

    






#endif
