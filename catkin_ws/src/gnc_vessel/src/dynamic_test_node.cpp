
// To test the dynamic of vessel

#include"ros/ros.h"
#include<memory>
#include"gnc_msgs/control.h"

std::unique_ptr<ros::Publisher> pubPtr_control;
std::unique_ptr<ros::Timer> Ptr_timer;

void timerCallback(const ros::TimerEvent&)
{
    gnc_msgs::control msg_control;
    msg_control.surge=100;
    msg_control.sway=0;
    msg_control.yaw=0;
    
    pubPtr_control->publish(msg_control);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_test_node");
    
    ros::NodeHandle nh;
    
    ros::Timer timer;
    
    double step_size=0.1;
    
//     std::unique_ptr<ros::Publisher> pubPtr_control;
//     std::unique_ptr<ros::Timer> Ptr_timer;
    
    
    
    pubPtr_control=std::unique_ptr<ros::Publisher>(new 
        ros::Publisher(nh.advertise<gnc_msgs::control>
        ("tug1/control", 1000)));
    
    timer=nh.createTimer(ros::Duration(step_size), &timerCallback);
    
    ros::spin();
    return 0;
    
}


