// // Jiang Xu
// jiangxvv@sjtu.edu.cn

// the ros node for the tug1_controller


#include"DPtugControllerInterface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tug1_controller");
  
  
  
  VesselControlProperty config;
  config.name="Tug1";
  config.mass=625229;
  config.Irr=config.mass*7.5*7.5;
  config.a11=7.58e4;
  config.a22=3.69e5;
  config.a23=-1.4e5;
  config.a33=8.77e6;
  config.d11=0.32;
  config.d22=0.637;
  config.d33=0.0437;
  
  DPtugControllerInterface Tug1_controller(config, config.name, 0.05, 5.0, 1.0);
  
  return Tug1_controller.run();
  
}