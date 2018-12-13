// Jiang Xu
// jiangxvv@sjtu.edu.cn

// The dynamic interface of tug

#ifndef DPTUGDYNAMIC_H
#define DPTUGDYNAMIC_H

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



class DPtugDynamic 
{
    // forward declare
    //class DPtug;
    
public:
    // constructor
    DPtugDynamic(const double& timer_step, const std::string& name);
    
    
    // public method
    int run()
    {
        ros::spin();
        return EXIT_FAILURE;
    }
    
    
private:
    // private fields
    
    // ros nodehandle
    ros::NodeHandle nh_;
    
    // timer which control simulation and the publish rate
    ros::Timer timer_;
    
    // publishers and messages
    std::unique_ptr<ros::Publisher> pubPtr_vel_;
    std::unique_ptr<ros::Publisher> pubPtr_pos_;
    
    geometry_msgs::Pose2D msg_pos_;
    geometry_msgs::Twist msg_vel_;
    
    // subscriber 
    std::unique_ptr<ros::Subscriber> subPtr_act_;
    
    // vehicle for the simulation
    std::unique_ptr<DPtug> Ptr_vehicle_;
    
    // time step of messages publishing
    double step_size_=0.05;
    
    // txtfile name of tug init_position 
    std::string name_;  // for example tug1
    
    
    
    bool received_control_;
    
    // private methods
    void get_init_position(const std::string&);
    void timerCallback(const ros::TimerEvent&);
    void callback_sub(const usv_towing::control& msg_ctrl);
    
    
};
#endif
