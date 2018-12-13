// Jiang Xu
// jiangxvv@sjtu.edu.cn

// node for the mothership dynamic node 

#include"gnc_vessel/vessel_dynamic_interface.h"
#include"gnc_vessel/vessel.h"
#include<string>
#include<ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mothership_dynamic" );
    
    VesselParameters config;
    config.name="mothership";
    config.mass=33400000;
    config.radius=33.54;
    config.a11=1.2E7;
    config.a22=3.3E7;
    config.a23=0;
    config.a33=2.8012E10;
    config.Xu=0.8;
    config.Yv=1.7;
    config.Nr=0;
    
    double timer_step=0.05;
    
    std::vector<double> init_velocity={0, 0, 0};
    std::vector<double> init_position={0, 0, 0};
    std::vector<double> init_actuation={0, 0, 0};
    
    VesselDynamicInterface mothership(config, timer_step, init_velocity,
                                      init_position, init_actuation);
    
    return mothership.run();
}
    
    