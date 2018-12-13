// Jiang Xu
// jiangxvv@sjtu.edu.cn

// The dynamic of mothership(class)

#ifndef  MOTHERSHIPDYNAMIC_H
#define MOTHERSHIPDYNAMIC_H

#include <ros/ros.h>
#include <memory>
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include "MotherShip.h"
#include <geometry_msgs/Pose2D.h>
#include "usv_towing/control.h"

class MotherShipDynamic
{
public:
    // constructor
    MotherShipDynamic(const double& timer_step);

    // public method
    int run()
    {
        ros::spin();
        return EXIT_FAILURE;
    }

private:
    // privare fields

    //ros node
    ros::NodeHandle nh_;

    // timer which controls simulation and the publishing rate
    ros::Timer timer_;

    // publishers and messages
    std::unique_ptr<ros::Publisher> pubPtr_vel_;
    std::unique_ptr<ros::Publisher> pubPtr_pos_;

    geometry_msgs::Pose2D msg_pos_;
    geometry_msgs::Twist msg_vel_;


    // subscriber 
    std::unique_ptr<ros::Subscriber> subPtr_act_;

    // vehicle for the simulation
    std::unique_ptr<MotherShip> Ptr_vehicle_;

    // time step of messages publishing
    double step_size_=0.05;

    // flag 
    bool received_control_;

    // private methods
    void get_ros_param();
    void timerCallback(const ros::TimerEvent&);
    void callback_sub(const usv_towing::control& msg_ctrl);


};

#endif