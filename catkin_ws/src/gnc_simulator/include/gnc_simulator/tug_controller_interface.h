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

// class of the central controller

#ifndef TUG_CONTROLLER_INTERFACE_H
#define TUG_CONTROLLER_INTERFACE_H

#include<ros/ros.h>
#include <memory>
#include<string>
#include <geometry_msgs/Vector3.h>
#include<geometry_msgs/Point.h>
#include"gnc_msgs/MotionStamped.h"
#include <eigen3/Eigen/Dense>

#include"gnc_controller/basic_controller.h"
#include"gnc_controller/feedback_linear_controller.h"




// class define
class TugControllerInterface
{
    
public:
    // constructor
    TugControllerInterface(const VesselControlProperty& vessel,
                           const double& vessel_length,
                           const double& time_step,
                           const double& kp, 
                           const double& k);
    
    // public methods
    int run()
    {
        ros::spin();
        return EXIT_FAILURE;
    }
            
    //virtual ~TugControllerInterface();
                    
private:
    // private field
    // ros node
    ros::NodeHandle nh_;
    
    // publisher and message of the actuation signal
    std::unique_ptr<ros::Publisher> pubPtr_actuation_;
    geometry_msgs::Vector3 msg_actuation_;
    
    // create a timer to control the publishing of control commands
    ros::Timer timer_;
    
    // time step 
    double time_step_;
    
    VesselControlProperty vessel_;
    double vessel_length_;
    std::string name_;
    double xg_;
    
    // subscribers    
    std::unique_ptr<ros::Subscriber> sub_touch_point_;
    std::unique_ptr<ros::Subscriber> sub_tug_motion_;
    std::unique_ptr<ros::Publisher> pubPtr_control_;
    
    std::unique_ptr<FeedbackLinearController> Ptr_controller_;

    
    // actuation computed by the controller
    Eigen::Vector3d actuation_;
    
    // helper variable
    bool received_course_info_=false;
    
    // parameters of controller
    double kp_, k_;
    
    // desired position of tug(gravity point)
    double xd_, yd_;
    
    // position and velocity of the vehicle
    double current_time_;
    Eigen::Vector3d position_, velocity_, acceleration_;
    
    // course variables
    Eigen::Vector3d pd_, pd1_, pd2_;
    
//     // errors of the motion
//     Eigen::Vector3d error_pose_, error_vel_, error_;
//     
//     // dynamic matrix
//     Eigen::Matrix3d mass_, CCmatrix_;
//     Eigen::Vector3d damping_;
//     
//     // rotation matrix
//     Eigen::Matrix3d R_, Rt_;
//     
//     // skew-symmetric matrix
//     Eigen::Matrix3d SS_;
    
    
    
    void timerCallback(const ros::TimerEvent&);
    
    void callback_motionstate(const gnc_msgs::MotionStamped& msg);
    void callback_course(const geometry_msgs::Point& msg);
    //         void get_ros_param();
    
public:
    Eigen::Vector3d ActuationCompute();
    
};

#endif