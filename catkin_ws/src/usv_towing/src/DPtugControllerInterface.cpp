// jiang xu
// jiangxvv@sjtu.edu.cn

// the interface of the dptug's controller
#include<iostream>
#include"DPtugControllerInterface.h"

DPtugControllerInterface::DPtugControllerInterface(
                            const BasicController::VesselControlProperty&config,
                            const std::string& name, const double& time_step, 
                            const double& kp, const double& k)
        :config_(config), name_(name), time_step_(time_step), kp_(kp), k_(k)
{
    
    
    // define the vehicle
    Ptr_controller_ = std::unique_ptr<FeedbackLinearController>(new           FeedbackLinearController(config_, step_size_, kp_, k_));
    // define the timer
    timer_=nh_.createTimer(ros::Duration(time_step_),
                           &DPtugControllerInterface::timerCallback, this);
    
    // define the publisher
    pubPtr_control_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<usv_towing::control>(name_+"/allocation",1000)));
    
    // define the subscriber
    sub_pos_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe(name_+"/pose",1000, &CentralController::callback_pose,this)));
    sub_vel_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe(name_+"/vel",1000, &CentralController::callback_vel,this)));
    sub_course_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe(name_+"/course",1000, &CentralController::callback_course,this)));
}



void DPtugControllerInterface::timerCallback(const ros::TimerEvent&)
{
    Act_output_=Ptr_contrller_->ComputeControl();
    
    msg_control_.surge=Act_output_(0);
    msg_control_.sway=Act_output_(1);
    msg_control_.yaw=Act_output_(1);
    
    pubPtr_control_->publish(msg_control_);
}


void DPtugControllerInterface::callback_pose(const geometry_msgs::Pose2D& msg_pose)
{
    Ptr_controller_->vessel_position<< msg_pose.x, msg_pose.y, msg_pose.theta;
}

void DPtugControllerInterface::callback_vel(const geometry_msgs::Twist& msg_vel)
{
    Ptr_controller_->vessel_velocity_<< msg_vel.linear.x, msg_vel.linear.y, msg_vel.angular.z;
}

void DPtugControllerInterface::callback_course(const usv_towing::course& msg_course)
{
    Ptr_controller_->course_position_<< msg_course.pose.x, msg_course.pose.y, msg_course.pose.z;
    
    Ptr_controller_->course_velocity_<< msg_course.velocity.x, msg_course.velocity.y,
    msg_course.velocity.z;
    
    Ptr_controller_->course_acceleration_<<msg_course.acceleration.x, msg_course.acceleration.y, msg_course.acceleration.z;
}




