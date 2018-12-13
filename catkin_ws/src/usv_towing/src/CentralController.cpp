// Jiang Xu 
// jiangxvv@sjtu.edu.cn

#include "CentralController.h"

#include<ros/ros.h>
#include<Eigen/Dense>
#include<memory>
#include<cmath>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include"usv_towing/control.h"
#include"usv_towing/course.h"
#include"MotherShip.h"


CentralController::CentralController(const double& time_step,
                            const double& kp=5.0, const double& k=1.0)
    :time_step_(time_step), kp_(kp), k_(k)
{   

    // define the vehicle
    Ptr_vehicle_ = std::unique_ptr<MotherShip>(new MotherShip());
    // define the timer
    timer_=nh_.createTimer(ros::Duration(time_step_),
        &CentralController::timerCallback, this);
    
    // define the publisher
    pubPtr_control_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<usv_towing::control>("ship/allocation",1000)));
    
    // define the subscriber
    sub_pos_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("ship/pose",1000, &CentralController::callback_pose,this)));
    sub_vel_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("ship/vel",1000, &CentralController::callback_vel,this)));
    sub_course_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("ship/course",1000, &CentralController::callback_course,this)));

    
}

CentralController::~CentralController(){}



void CentralController::timerCallback(const ros::TimerEvent&)
{   
    // calculate the actuation
    actuation_=ActuationCompute();

    // set the actuation signal
    msg_actuation_.surge=actuation_[0];
    msg_actuation_.sway=actuation_[1];
    msg_actuation_.yaw=actuation_[2];

    // publish the message
    pubPtr_control_->publish(msg_actuation_);

}

Eigen::Vector3d CentralController::ActuationCompute()
{
    mass_=Ptr_vehicle_->InertiaMatrix();
    CCmatrix_=Ptr_vehicle_->InertiaCCMatrix(velocity_);
    damping_=Ptr_vehicle_->DampingVector(velocity_);

    R_=Ptr_vehicle_->RotationMatrix(position_);
    Rt_=R_.transpose();

    SS_<<0, -r_, 0, r_, 0, 0, 0, 0, 0;

    error_pose_=Rt_*(position_-pd_);
    error_vel_=Rt_*pd1_-velocity_;
    error_=error_vel_+kp_*error_pose_;

    Eigen::Vector3d Force;
    Force=CCmatrix_*velocity_+damping_+error_pose_+k_*error_
        +mass_*(-SS_*(Rt_*pd1_+kp_*error_pose_)+Rt_*pd2_+kp_*error_vel_);
    
    return Force;

}
