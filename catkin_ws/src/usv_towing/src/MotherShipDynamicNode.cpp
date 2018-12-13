// Jiang Xu
// jiangxvv@sjtu.edu.cn

// The node of mothership dynamic

#include "MotherShipDynamic.h"
#include<ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MotherShip");
    
    MotherShipDynamic MotherShip(0.05);

    return MotherShip.run();
}