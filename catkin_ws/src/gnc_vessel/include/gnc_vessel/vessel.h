// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class of vehicle, which define the basic properties.

#ifndef VESSEL_H
#define VESSEL_H

#include<Eigen/Dense>
#include<array>
#include<vector>
#include<string>

struct VesselParameters
{
  std::string name;
  double mass;
  double radius;
  double length;
  double a11;
  double a22;
  double a23;
  double a33;
  double Xu;
  double Yv;
  double Nr;
};


class Vessel;
void RunVessel(Vessel&, const size_t&, const double&);

class Vessel
{
  friend class BasicController;
  friend void RunVessel(Vessel&, const size_t&, const double&);

public:
  Vessel()=default;
  
  Vessel(const VesselParameters& config, 
	      const std::vector<double>& init_velocity,
	      const std::vector<double>& init_position,
	      const std::vector<double>& init_actuation);
  
  ~Vessel() {}
  
  
public: //protected will be error in the vessel_dynamic_interface.cpp
  
  // new matrix typedef
  typedef Eigen::Matrix<double, 6, 1>  Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  
  std::string name_;  // name of vehicle
  
  // physical parameter of the vehicle
  
  double mass_;
  double radius_;
  double length_;  // length of the vessel
  double Izz_;
  double a11_;
  double a22_;
  double a23_;
  double a33_;
  double Xu_;
  double Yv_;
  double Nr_;
  
  
  const double GRAVITY_=9.81;
  const double pi_=3.1416;
  const double rou_=1025;

  
  // motion state of the vehicle
  Eigen::Vector3d velocity_;
  Eigen::Vector3d position_;
  Eigen::Vector3d acceleration_;
  
  Eigen::Vector3d actuation_;
  
//   std::vector<double> 
  
  //initial motion state of the vehicle
  std::vector<double> init_velocity_, init_position_, init_actuation_;
  // array of the velocity and position vectors during the simualation
  std::vector<Eigen::Vector3d> velocity_history_, position_history_;
  
  // parameters of the simulation
  double step_size_=0.0;
  size_t step_number_=0;
  size_t step_counter_=0;
  double current_time_=0.0;
  std::vector<double> time_vec_={0};  
  
  // private methods 
  
  // rotation matrix
  Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& pose) const;
  
  //inertia matrix of the vehicle
  Eigen::Matrix3d InertiaMatrix( ) const;
  
  // inertia centripetal and coriolis matrix of the vehicle
  Eigen::Matrix3d InertiaCCMatrix(const Eigen::Vector3d&) const;
  
  // damping vector of the vehicle
virtual  Eigen::Vector3d DampingVector(const Eigen::Vector3d&) const;
  
  Vector6d StateDerivative(const Eigen::Vector3d& velocity,
			   const Eigen::Vector3d& position,
			   const double t) const;
			   
   // operation function to integrate (function object)
  inline void operator() (const std::vector<double>& state,
			  std::vector<double>& state_derivative, const double);
  
};






#endif