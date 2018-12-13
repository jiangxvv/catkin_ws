// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class for vessel dynamic interface

#ifndef VESSELDYNAMICINTERFACE_H
#define VESSELDYNAMICINTERFACE_H

#include"gnc_vessel/vessel.h"

#include<Eigen/Dense>
#include<vector>
#include<memory>
#include<cstdlib>

#include<ros/ros.h>
#include"gnc_msgs/MotionStamped.h"
// #include<geometry_msgs/Pose2D.h>
// #include<geometry_msgs/Twist.h>
#include"gnc_msgs/control.h"

class VesselDynamicInterface
{
public:
    VesselDynamicInterface(const VesselParameters& vessel, const double& timer_step,
                           const std::vector<double>& init_velocity,
                           const std::vector<double>& init_position,
                           const std::vector<double>& init_actuation);
    
    // public method
    int run()
    {
        ros::spin();
        return EXIT_FAILURE;
    }
    
private:
    // privare fields
    
    VesselParameters vessel_;
    
    std::string name_;
    
    //ros node
    ros::NodeHandle nh_;
    
    // timer which controls simulation and the publishing rate
    ros::Timer timer_;
    
    //initial motion state of the vehicle
    std::vector<double> init_velocity_, init_position_, init_actuation_;
    
    // publishers and messages
    std::unique_ptr<ros::Publisher> pubPtr_motion_state_;
    gnc_msgs::MotionStamped msg_motion_state_;
//     std::unique_ptr<ros::Publisher> pubPtr_pos_;
//     
//     geometry_msgs::Pose2D msg_pos_;
//     geometry_msgs::Twist msg_vel_;
    
    
    // subscriber 
    std::unique_ptr<ros::Subscriber> subPtr_act_;
    
    // vehicle for the simulation
    std::unique_ptr<Vessel> Ptr_vessel_;
    
    // time step of messages publishing
    double step_size_=0.05;
    
    // flag 
    bool received_control_;
    
    // private methods
    void timerCallback(const ros::TimerEvent&);
    void callback_sub(const gnc_msgs::control& msg_ctrl);
    

};

#endif 