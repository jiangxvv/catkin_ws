// Jiang Xu
// jiangxvv@sjtu.edu.cn

// the interface of the tug dynamic

#include"DPtugDynamic.h"

#include<ros/ros.h>
#include<Eigen/Dense>
#include<memory>
#include<string>
#include<fstream>
#include<cstdlib>
#include"DPtug.h"

#include"geometry_msgs/Pose2D.h"
#include"geometry_msgs//Twist.h"
#include"usv_towing/control.h"

DPtugDynamic::DPtugDynamic(const double& timer_step, const std::string& name)
                :step_size_(timer_step), name_(name)
{
    
    // define the timer
    timer_=nh_.createTimer(ros::Duration(step_size_), &DPtugDynamic::timerCallback, this);
    
    // message name of tug
    std::string tug_vel=name_+"/vel";    /* for example tug1/vel */
    std::string tug_pose=name_+"/pose";  /* for example tug1/pose */
    std::string tug_act=name_+"/act";
    
    // define the publishers
    pubPtr_vel_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Twist>(tug_vel,1000)));
    
    pubPtr_pos_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<geometry_msgs::Pose2D>(tug_pose,1000)));
    
    // define the subscriber
    subPtr_act_=std::unique_ptr<ros::Subscriber>(new ros::Subscriber(nh_.subscribe(
        tug_act, 1000, &DPtugDynamic::callback_sub, this)));
    
    // get the initial position and define the tug
    get_init_position(name_);
    
}

// get the initial position and define the tug
void DPtugDynamic::get_init_position(const std::string& name)
{
    // load the initial position of the tug
    std::string txtname="home/jiangxvv/catkin_ws/src/usv_towing/cfg/"+name+"_init_position.txt";
    std::fstream file;
    double temp;
    std::vector<double> init_position;
    file.open(txtname);
    while(!file.eof())
    {
        file>>temp;
        init_position.push_back(temp);
    }
    init_position.pop_back(); // upper read the last number twice, so pop_back
    
    Ptr_vehicle_=std::unique_ptr<DPtug>(
        new DPtug({0, 0, 0}, init_position, {0 ,0 ,0}));
    
}



// define the callback function of the timer
void DPtugDynamic::timerCallback(const ros::TimerEvent&)
{
    // publish the motion information of the tug
    ROS_INFO("Time: %5.2f s, Main thrust: %5.2f N, Yaw moment: %5.2f Nm",
             Ptr_vehicle_->current_time_, Ptr_vehicle_->actuation_[0],
             Ptr_vehicle_->actuation_[2]);
    //ROS_INFO("Iime: %5.2f s, Main_thruster: %5.2f N, X: %5.2f m, y: %5.2f m, angle:%5.2f，
        //Ptr_vehicle_->current_time_, Ptr_vehicle_->actation_[0], 
        //Ptr_vehicle_->position_[0], Ptr_vehicle_->position_[1],
        //Ptr_vehicle_->position_[2】")；
    
    msg_vel_.linear.x = Ptr_vehicle_->velocity_[0];
    msg_vel_.linear.y = Ptr_vehicle_->velocity_[1];
    msg_vel_.linear.z = 0;
    msg_vel_.angular.x = 0;
    msg_vel_.angular.y = 0;
    msg_vel_.angular.z = Ptr_vehicle_->velocity_[2];

    msg_pos_.x = Ptr_vehicle_->position_[0];
    msg_pos_.y = Ptr_vehicle_->position_[1];
    msg_pos_.theta = Ptr_vehicle_->position_[2];
    
    pubPtr_vel_->publish(msg_vel_);
    pubPtr_pos_->publish(msg_pos_);
    
    if (!received_control_)
    {
        Ptr_vehicle_->actuation_[0]=0;
        Ptr_vehicle_->actuation_[1]=0;
        Ptr_vehicle_->actuation_[2]=0;
    }
    
    RunVehicle(*Ptr_vehicle_, 1, step_size_);
    
    received_control_=false;
}

void DPtugDynamic::callback_sub(const usv_towing::control& msg_ctrl)
{
    received_control_=true;
    
    Ptr_vehicle_->actuation_[0]=msg_ctrl.surge;
    Ptr_vehicle_->actuation_[1]=msg_ctrl.sway;
    Ptr_vehicle_->actuation_[2]=msg_ctrl.yaw;
}


























    

                              

