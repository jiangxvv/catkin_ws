// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class of the DPtug

#ifndef DPTUG_H
#define DPTUG_H

#include<cmath>
#include<string>
#include<vector>
#include<Eigen/Dense>
#include<boost/math/constants/constants.hpp>
#include<boost/numeric/odeint.hpp>


// forward declaration
class DPtug;
void RunVehicle(DPtug&, const size_t&, const double&);

// class DPtug
class DPtug
{
  // friend class and function
  friend class DPcontroller;
  friend void RunVehicle( DPtug &, const std::size_t &, const double & );
  
  // constant pi
  const double pi=boost::math::constants::pi<double>();
  
public:
  // constructor
  DPtug(const std::vector<double>& init_velocity,
                const std::vector<double>& init_position,
                const std::vector<double>& init_thrust);
  DPtug() :DPtug({1.0, 0, 0}, {0, 0, 0}, {0, 0, 0}) {}
  DPtug(const std::vector<double>& init_velocity)
            :DPtug(init_velocity, {0, 0, 0}, {0, 0, 0}) {}
  DPtug(const std::vector<double>& init_velocity,
          const std::vector<double>& init_position)
          :DPtug(init_velocity,init_position, {0, 0, 0}) {}

private:
  // private fields
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  
  // parameters of the vehicle
  const double gravity_ = 9.81;
  const double mass_ = 180;
  const double Izz_ = 250;
  const double a11_ = 27;
  const double a22_ = 37;
  const double a33_ = 93.2345;
  const double a23_ = 50;
  const double Xu_ = 0;
  const double Yv_ = 300;
  const double Yr_ = -40;
  
  double length_=100;
  double width_=100;
  
  // parameters of the simulation
  // step size of the current simulation
  double step_size_ = 0.0;
  
  // step number of the simulation
  size_t step_number_=0;
  
  // counter of step
  size_t step_counter_=0;
  
  // time vector of the simulation
  std::vector<double> time_vec_={0};
  
  // array of velocity and position vectors during the simulation
  std::vector<Eigen::Vector3d> velocity_history_, position_history_;
  
  // privatie methods
  
  // rotation matrix declaration
  Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& pose) const;
  
  // inertia matrix of the vehicle
  Eigen::Matrix3d InertiaMatrix() const;
  
  // inertia centripetal and coriolis matrix of the vehicle
  Eigen::Matrix3d InertiaCCMatrix(const Eigen::Vector3d&) const;
  
  // damping vector of the vehicle
  Eigen::Vector3d DampingVector(const Eigen::Vector3d&) const;
  
  // time derivative of the system state
  Vector6d StateDerivative(const Eigen::Vector3d& velocity,
			   const Eigen::Vector3d& position,
			   const double/* t */) const;
			   
			   
  
public:
  // public fields
  
  // elapsed time since current simulation starts
  double current_time_=0;
  
  // initial velocity and position of the vehicle
  Eigen::Vector3d init_velocity_, init_position_, init_actuation_;
  
  // velocity and position of the vehicle,   actuation
  Eigen::Vector3d velocity_, position_, actuation_;
  
  // cater position and force angle related to the mothership
  double x_, y_, alpha_;
  
  // length of cable
  double cable_length_;
  
  // operator function to integrate
  void operator()(const std::vector<double>& state,
		  std::vector<double>& state_derivative, const double);
};

#endif // DPtug_H
  
  
  
  
  
  
  
  
  
  
  
  
  
  

