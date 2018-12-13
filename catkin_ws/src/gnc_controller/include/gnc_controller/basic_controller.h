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

#ifndef BASIC_CONTROLLER_H
#define BASIC_CONTROLLER_H

#include<Eigen/Dense>
#include<string>
#include<iostream>

struct VesselControlProperty
{
    std::string name;
    double mass = 0;
    double radius=0;
    double length=0;
   // double Irr = 0;
    double a11 = 0;
    double a22 = 0;
    double a23 = 0;
    double a33 = 0;
    double xg = 0;
    double d11 = 0;
    double d22 = 0;
    double d33 = 0;
};
 
// class BasicController 
// BasicController is a base class, which defines the basic controller structures

class BasicController
{
protected:
    
    // vessel parameters
  const double GRAVITY_ = 9.81;
  const double rou_=1025; /*density of water */
  double mass_ = 0; /**< vessel mass */
  double radius_=0;
  double Irr_ = mass_*radius_*radius_;  /**< inertia moment */
  double length_=0; /* the length of the vessel */
  double a11_ = 0;  /**< added mass in surge */
  double a22_ = 0;  /**< added mass in sway */
  double a23_ = 0;  /**< added mass cross term */
  double a33_ = 0;  /**< added mass in yaw */
  double xg_ = 0;   /**< longitudinal coordinate of gravity center */
  double d11_ = 0;  /**< linear damping coefficient in surge */
  double d22_ = 0;  /**< linear damping coefficient in sway */
  double d33_ = 0;  /**< linear damping coefficient in yaw */

  
  // vessel states
  Eigen::Vector3d vessel_position_;     /**< vessel coordinate and heading */
  Eigen::Vector3d vessel_velocity_;     /**< vessel linear velocity and yaw rate */
  Eigen::Vector3d vessel_acceleration_ ; /**< vessel acceleration */
  
  // desire course states
  Eigen::Vector3d course_position_;
  Eigen::Vector3d course_velocity_;  // the derivate of the position in the earth_fixed frame 
  Eigen::Vector3d course_acceleration_;
  
  // state error between the vessel and the course
  Eigen::Vector3d error_pose_, error_vel_, error_acc_;
  
  // control output in surge, sway, yaw
  Eigen::Vector3d output_;
  
  // step size 
  double step_size_=0;
  
  
  // 
  Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& pose) const;
  
  // inertia matrix of the vehicle
  Eigen::Matrix3d InertiaMatrix() const;
  
  // inertia centripetal and coriolis matrix of the vehicle
  Eigen::Matrix3d InertiaCCMatrix(const Eigen::Vector3d&) const;
  
  // damping vector of the vehicle
  Eigen::Vector3d DampingVector(const Eigen::Vector3d&) const;
  
  
  
public:
    // brief constructor
    BasicController(const VesselControlProperty& vessel, const double& step_size);
    
    // brief a destructor
    ~BasicController(){}
    
    // return control output
    Eigen::Vector3d getOutput() const
    { 
        return output_;
    }
    
    // return time step size of the controller
    double getStepSize() const 
    {
        return step_size_;
    }
    
    // set current position of vessel
    void setVesselPosition(const Eigen::Vector3d& pose)
    {
        vessel_position_=pose;
    }
    
    // set current velocity of vessel
    void setVesselVelocity(const Eigen::Vector3d& vel)
    {
        vessel_velocity_=vel;
    }
    
    void setVesselAcceleration(const Eigen::Vector3d& acc)
    {
        vessel_acceleration_=acc;
    }
    
    
    // set the position of course
    void setCoursePosition(const Eigen::Vector3d& pose)
    {
        course_position_=pose;
    }
    
    
    // set the velocity of course
    void setCourseVelocity(const Eigen::Vector3d& vel)
    {
        course_velocity_=vel;
    }
    
    
    // set the acceleration of course
    void setCourseAcceleration(const Eigen::Vector3d& acc)
    {
        course_acceleration_=acc;
    }
  
};


#endif
