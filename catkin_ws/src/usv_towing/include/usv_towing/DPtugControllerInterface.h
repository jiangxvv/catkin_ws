// Jiang Xu
// jiangxvv@sjtu.edu.cn

// the interface of the dptug's controller

#ifndef DPTUGCONTROLLERINTERFACE_H
#define DPTUGCONTROLLERINTERFACE_H

#include<iostream>
#include<Eigen/Dense>
#include<ros/ros.h>
#include<memory>

#include<geometry_msgs/Pose2D.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Vector3.h>
#include"usv_towing/control.h"
#include"usv_towing/course.h"

#include"BasicController.h"
#include"FeedbackLinearController.h"

class DPtugControllerInterface
{
public:
    DPtugControllerInterface(const VesselControlProperty& config,
                             std::string& name, const double& time_step, 
                             const double& kp, const double& k);
                         
    int run()
    {
        ros::spin();
        return EXIT_FAILURE;
    }
    
private:
    ros::NodeHandle nh_;
    
    double kp_, k_;
    
    std::string name_;
    
    Eigen::Vector3d Act_output_;
    
    BasicController::VesselControlProperty config_;
    
    std::unique_ptr<ros::Publisher> pubPtr_control_;
    usv_towing::control msg_actuation_;
    
    // create a timer control the publishing of  control commands
    ros::Timer timer_;
    
    // time_step
    double time_step_;
    
    // subscribers
    std::unique_ptr<ros::Subscriber> sub_pos_;
    std::unique_ptr<ros::Subscriber> sub_vel_;
    std::unique_ptr<ros::Subscriber> sub_course_;
    
    // controller ptr
    std::unique_ptr<FeedbackLinearController> Ptr_controller_;
    
    
    void callback_pose(const geometry_msgs::Pose2D& msg_pose);
    void callback_vel(const geometry_msgs::Twist& msg_vel);
    void callback_vel(const usv_towing::course& msg_course);
    void timerCallback(const ros::TimerEvent&);
                         
}


#endif