// Jiang Xu
// jiangxvv@sjtu.edu.cn

// ros node of central controller

#include "CentralController.h"
#include<ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CentralController");

    CentralController CentralController(0.1, 5.0, 1.0);
    
    return CentralController.run();

}