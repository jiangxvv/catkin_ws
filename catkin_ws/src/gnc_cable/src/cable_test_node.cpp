

#include<ros/ros.h>
#include"cable.h"
// #include"gnc_cable/cable.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cable_test_node");
    ros::NodeHandle nh;
    
    
    double length=400;
    Cable cable(length);
    double force=cable.conputeForce(499.5);
    std::cout<<force<<std::endl;
    
    //ros::spin();
    return 0;
}