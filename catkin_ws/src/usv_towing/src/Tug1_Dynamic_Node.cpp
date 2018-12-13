// Jiang Xu
// jiangxvv@sjtu.edu.cn

// node of tug1's dynamic

#include<ros/ros.h>
#include"DPtugDynamic.h"
#include<string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Tug1");
    
    //const std::string name="tug1";
    
    DPtugDynamic tug1_dynamics(0.05, "tug1");
    
    return tug1_dynamics.run();
}
    

