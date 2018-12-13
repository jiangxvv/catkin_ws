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
// for

#ifndef CENTRAL_CONTROLLER_INTERFACE_H
#define CENTRAL_CONTROLLER_INTERFACE_H

#include<ros/ros.h>
#include <memory>
#include"gnc_msgs/MotionStamped.h"
#include <geometry_msgs/Point.h>
#include"gnc_msgs/course.h"
#include <eigen3/Eigen/Dense>

#include"gnc_controller/feedback_linear_controller.h"
#include"gnc_controller/basic_controller.h"
#include"gnc_controller/variable_thruster_allocation.h"
#include"gnc_cable/cable.h"



// class define
class CentralControllerInterface
{
        
public:
// constructor
        CentralControllerInterface(const VesselControlProperty& vessel,
                                   const double& time_step,const double& kp, 
                                   const double& k,const double& cable_length,
                                   const Eigen::Vector3d& touch_position1,
                                   const Eigen::Vector3d& touch_position2);

        // public methods
        int run()
        {
                ros::spin();
                return EXIT_FAILURE;
        }
        
        //~CentralControllerInterface();

private:
        // private field
        // ros node
        ros::NodeHandle nh_;

        // publisher and message of the actuation signal
        std::unique_ptr<ros::Publisher> pubPtr_actuation_;
        
        // publish the desided position of tug1 and tug2;
        std::unique_ptr<ros::Publisher> pubPtr_pd1_; 
        std::unique_ptr<ros::Publisher> pubPtr_pd2_;
        geometry_msgs::Point tug1_pd; // position desided of tug1
        geometry_msgs::Point tug2_pd; // position desided of tug2
        

        // create a timer to control the publishing of control commands
        ros::Timer timer_;

        // time step 
        double time_step_;
        double cable_length_;
        Eigen::Vector3d touch_position1_;
        Eigen::Vector3d touch_position2_;
        
        // force and angle
        double f1_;
        double f2_;
        double alpha1_;
        double alpha2_;
        
        
        VesselControlProperty vessel_;

        // subscribers
        std::unique_ptr<ros::Subscriber> sub_course_;
        std::unique_ptr<ros::Subscriber> sub_motion_state_;
        
        
        std::unique_ptr<FeedbackLinearController> Ptr_controller_;
        std::unique_ptr<VariableThrusterAllocation> Ptr_allocation_;
        std::unique_ptr<Cable> Ptr_cable_;
        
        // motion state of mothership
        double current_time_;
        Eigen::Vector3d pose_;
        Eigen::Vector3d velocity_;
        Eigen::Vector3d acceleration_;
        
        // desired position of tug1 and tug2 in global coordinate (x0,y0)
        Eigen::Vector2d position1_;
        Eigen::Vector2d position2_;
               
        geometry_msgs::Point msg_tug1_pose;
        geometry_msgs::Point msg_tug2_pose;

//         // mothership class
//         std::unique_ptr<MotherShip> Ptr_vehicle_;
        
        // actuation computed by the controller
        Eigen::Vector3d actuation_;
        Eigen::Vector4d tug_force_;
        

        // helper variable
        bool received_course_info_=false;
        
        // parameters of controller
        double kp_, k_;
        

        void timerCallback(const ros::TimerEvent&);
        void callback_motionstate(const gnc_msgs::MotionStamped&);
        void callback_course(const gnc_msgs::course& msg_course);
        
        
//         void get_ros_param();

public:
    
        Eigen::Vector3d ActuationCompute();

};

#endif