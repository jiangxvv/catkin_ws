// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class for the vessel

#include"gnc_vessel/vessel_dynamic_interface.h"
#include"gnc_vessel/vessel.h"

#include <ros/ros.h>

#include "gnc_msgs/control.h"
#include<string>


VesselDynamicInterface::VesselDynamicInterface(const VesselParameters &vessel, 
             const double &timer_step, const std::vector< double > &init_velocity, 
             const std::vector< double > &init_position, 
             const std::vector< double> &init_actuation)
            :vessel_(vessel), step_size_(timer_step), init_velocity_(init_velocity),
             init_position_(init_position), init_actuation_(init_actuation)
{
    name_=vessel_.name;
    
    // define the timer
    timer_=nh_.createTimer(ros::Duration(step_size_), 
                          &VesselDynamicInterface::timerCallback, this);
    
    std::string msg_name=name_+"/motion_state";
    
    
//     std::string pose_msg_name=name_+"/pose";
    std::string act_msg_name=name_+"/control";
    
    //define the publishers
    pubPtr_motion_state_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<gnc_msgs::MotionStamped>(msg_name, 1000)));

    
    // define the subscriber
    subPtr_act_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe(act_msg_name, 1000, &VesselDynamicInterface::callback_sub,this)));
    
    Ptr_vessel_=std::unique_ptr<Vessel>(new Vessel(vessel_, init_velocity_,
                                       init_position_, init_actuation_));
  
}

void VesselDynamicInterface::timerCallback(const ros::TimerEvent &)
{
    // publish the motion information of the vehicle
    ROS_INFO("Time: %5.2f s, Yaw angle: %5.2f",
             Ptr_vessel_->current_time_, Ptr_vessel_->position_[2]);
    msg_motion_state_.time=Ptr_vessel_->current_time_;
    
    msg_motion_state_.vel.x=Ptr_vessel_->velocity_[0];
    msg_motion_state_.vel.y=Ptr_vessel_->velocity_[1];
    msg_motion_state_.vel.z=Ptr_vessel_->velocity_[2];

    msg_motion_state_.pose.x=Ptr_vessel_->position_[0];
    msg_motion_state_.pose.y=Ptr_vessel_->position_[1];
    msg_motion_state_.pose.z=Ptr_vessel_->position_[2];
    
    msg_motion_state_.accel.x=Ptr_vessel_->acceleration_[0];
    msg_motion_state_.accel.y=Ptr_vessel_->acceleration_[1];  
    msg_motion_state_.accel.z=Ptr_vessel_->acceleration_[2];
    pubPtr_motion_state_->publish(msg_motion_state_);
    
    // run one-step simulation
    if (!received_control_)
    {
        Ptr_vessel_->actuation_[0]=0;
        Ptr_vessel_->actuation_[1]=0;
        Ptr_vessel_->actuation_[2]=0;
    }
    RunVessel(*Ptr_vessel_, 1, step_size_);
        
    received_control_=false;
}


void VesselDynamicInterface::callback_sub(const gnc_msgs::control &msg_ctrl)
{
    Ptr_vessel_->actuation_[0] = msg_ctrl.surge;
    Ptr_vessel_->actuation_[1] = msg_ctrl.sway;
    Ptr_vessel_->actuation_[2] = msg_ctrl.yaw;
}





