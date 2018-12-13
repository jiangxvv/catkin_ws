// Jiang Xu
// jiangxvv@sjtu.edu.com
// the interface of mothership_dynamic

#include "MotherShipDynamic.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "MotherShip.h"
#include "usv_towing/control.h"


// constructor
MotherShipDynamic::MotherShipDynamic(const double& timer_step)
                                    : step_size_(timer_step)
{   
    // define the timer
    timer_=nh_.createTimer(ros::Duration(step_size_), &MotherShipDynamic::timerCallback, this);

    //define the publishers
    pubPtr_vel_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Twist>("mothership/vel",1000)));
    pubPtr_pos_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Pose2D>("mothership/pose",1000)));
    

    // define the subscriber
    subPtr_act_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(
        nh_.subscribe("mothership/actuation",1000, &MotherShipDynamic::callback_sub,this)));
    
    // get the ros parameters and define the vehicle
    get_ros_param();
}
//MotherShipDynamic::~MotherShipDynamic();


// get the ros parameters and define the vehicle
void MotherShipDynamic::get_ros_param()
{
  // define parameters for the ship's initial position
  const std::string PARAM_1 = "/ctrl_params/initialPosition/x";
  const std::string PARAM_2 = "/ctrl_params/initialPosition/y";
  const std::string PARAM_3 = "/ctrl_params/initialPosition/psi";

  double init_x, init_y, init_heading;

  bool ok_1 = ros::param::get(PARAM_1, init_x);
  bool ok_2 = ros::param::get(PARAM_2, init_y);
  bool ok_3 = ros::param::get(PARAM_3, init_heading);

  if (ok_1 && ok_2 && ok_3)
  {
    Ptr_vehicle_ = std::unique_ptr<MotherShip>(
        new MotherShip({0, 0, 0}, {init_x, init_y, init_heading}, {0, 0, 0}));
  }
  else
  {
    ROS_WARN("Initial position cannot be loaded. Use default values.");
    Ptr_vehicle_ = std::unique_ptr<MotherShip>(new MotherShip());
  }
}

void MotherShipDynamic::timerCallback(const ros::TimerEvent&)
{
    // publish the motion information of the vehicle
    ROS_INFO("Time: %5.2f s, Yaw angle: %5.2f",
            Ptr_vehicle_->current_time_, Ptr_vehicle_->position_[2]);
    msg_vel_.linear.x=Ptr_vehicle_->velocity_[0];
    msg_vel_.linear.y=Ptr_vehicle_->velocity_[1];
    msg_vel_.angular.z=Ptr_vehicle_->velocity_[2];
    msg_vel_.linear.z = 0;
    msg_vel_.angular.x = 0;
    msg_vel_.angular.y = 0;

    msg_pos_.x=Ptr_vehicle_->position_[0];
    msg_pos_.y=Ptr_vehicle_->position_[1];
    msg_pos_.theta=Ptr_vehicle_->position_[2];

    // run one-step simulation
    if (!received_control_)
    {
        Ptr_vehicle_->actuation_[0]=0;
        Ptr_vehicle_->actuation_[1]=0;
        Ptr_vehicle_->actuation_[2]=0;
    }
    RunVehicle(*Ptr_vehicle_, 1, step_size_);

    received_control_=false;

}

void MotherShipDynamic::callback_sub(const usv_towing::control& msg_ctrl)
{
    //received_control_=ture;

    Ptr_vehicle_->actuation_[0] = msg_ctrl.surge;
    Ptr_vehicle_->actuation_[1] = msg_ctrl.sway;
    Ptr_vehicle_->actuation_[2] = msg_ctrl.yaw;
}