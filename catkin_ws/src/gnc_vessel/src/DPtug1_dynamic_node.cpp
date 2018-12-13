// Jiang Xu
// jiangxvv@sjtu.edu.cn

// node for the DPtug1 dynamic node 

#include"gnc_vessel/vessel_dynamic_interface.h"
#include"gnc_vessel/vessel.h"
#include<string>
#include<ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tug1_dynamic");
    
    VesselParameters config;
    config.name="tug1";
    config.mass=625229;
    config.radius=7.5;
    config.a11=7.58E4;
    config.a22=3.69E5;
    config.a23=-1.4E5;
    
    config.a33=8.44E6;
    config.Xu=0.32;
    config.Yv=0.637;
    config.Nr=0.0437;
    
    double timer_step=0.05;
    
    std::vector<double> init_velocity={463.271, 32.5, 0};
    std::vector<double> init_position={0, 0, 0};
    std::vector<double> init_actuation={0, 0, 0};
    
    VesselDynamicInterface DPtug1(config, timer_step, init_velocity,
                                      init_position, init_actuation);
    
    return DPtug1.run();
}
    