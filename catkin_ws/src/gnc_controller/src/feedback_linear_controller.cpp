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
#include<Eigen/Dense>
#include"basic_controller.h"
#include"feedback_linear_controller.h"

FeedbackLinearController::FeedbackLinearController(const VesselControlProperty &vessel, 
                                                   const double &step_size, const double &kp, const double &k)
       :BasicController(vessel, step_size), kp_(kp), k_(k)
{
    
}




Eigen::Vector3d FeedbackLinearController::ComputeControl()
{
    //the parameters of the equation, M, C, D, here D is a vector.
    Eigen::Matrix3d M=InertiaMatrix();
    Eigen::Matrix3d C=InertiaCCMatrix(vessel_velocity_);
    Eigen::Vector3d D=DampingVector(vessel_velocity_);
    
    double r=vessel_velocity_[2];// yaw vessel_velocity_
    Eigen::Matrix3d SS;
    SS<<0, -r, 0, r, 0, 0, 0, 0, 0;
    
    
    // rotation matrix
    Eigen::Matrix3d R=RotationMatrix(vessel_position_);
    Eigen::Matrix3d Rt=R.transpose();
    
    error_pose_=-Rt*(vessel_position_-course_position_);
    error_vel_=Rt*course_velocity_-vessel_velocity_;
    Eigen::Vector3d error=error_vel_+kp_*error_pose_;
    
    Eigen::Vector3d output;
    output=C*vessel_velocity_+D+error_pose_+k_*error
            +M*(-SS*(Rt*course_velocity_+kp_*error_pose_)+Rt*course_acceleration_
            +kp_*error_vel_);
            
    output_=output;
    return output;
                  
}

