// Jiang Xu
// jiangxvv@sjtu.edu.cn
// The class of mothership to be towed

#ifndef MOTHERSHIP_H
#define MOTHERSHIP_H

#include<cmath>
#include<vector>
#include<Eigen/Dense>
#include<boost/numeric/odeint.hpp>
#include<string>
#include<boost/math/constants/constants.hpp>

// forward declaration
class MotherShip;
void RunVehicle(MotherShip&, const size_t&, const double&);

// class MotherShip
class MotherShip
{
    friend class CentralController;
    friend void RunVehicle(MotherShip&, const size_t&, const double&);

    const double pi=boost::math::constants::pi<double>();

public:

    // constructor
    MotherShip(const std::vector<double>& init_velocity,
                const std::vector<double>& init_position,
                const std::vector<double>& init_thrust);
    MotherShip() :MotherShip({1.0, 0, 0}, {0, 0, 0}, {0, 0, 0}) {}
    MotherShip(const std::vector<double>& init_velocity)
            :MotherShip(init_velocity, {0, 0, 0}, {0, 0, 0}) {}
    MotherShip(const std::vector<double>& init_velocity,
          const std::vector<double>& init_position)
          :MotherShip(init_velocity,init_position, {0, 0, 0}) {}

private:
    // private fields
    
    // new matrix types
    typedef Eigen::Matrix<double,6,1> Vector6d;
    typedef Eigen::Matrix<double,6,6> Matrix6d;

    // parameters of the mothership (need to be added later)
    const double gravity_=9.81;
    const double mass_= 3.34e7 ;
    const double radius_=33.54;
    const double Izz_ = mass_*radius_*radius_ ;
    const double a11_ = 1.2e7;
    const double a22_ = 3.3e7;
    const double a33_ = 2.8012e10;
    const double a23_ = 0;
    const double Xu_= 0.8;
    const double Yv_= 1.7;
    const double Yr_= 0;
    const double Nr_=0;
    

    // parameters of the simulation
    double step_size_=0.0;
    size_t step_number_=0;
    size_t step_counter_=0;

    // time vector of the simulation 
    std::vector<double> time_vec_={0};

    // array of the velocity and position vectors during the simualation
    std::vector<Eigen::Vector3d> velocity_history_, position_history_;
    
    // private methods 

    // rotation matrix
    Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& pose) const;

    //inertia matrix of the vehicle
    Eigen::Matrix3d InertiaMatrix( ) const;

    // inertia centripetal and coriolis matrix of the vehicle
    Eigen::Matrix3d InertiaCCMatrix(const Eigen::Vector3d&) const;

    // damping vector of the vehicle
    Eigen::Vector3d DampingVector(const Eigen::Vector3d&) const;

public:
    // public fields
    
    // elapsed time since current simulation starts
    double current_time_=0.0;

    // initial velocity and position of the vehicle
    std::vector<double> init_velocity_, init_position_, init_thruster_;

    // current velocity and position of the vehicle
    Eigen::Vector3d velocity_, position_;

    // actuation action on the vehicle
    Eigen::Vector3d actuation_;

    Vector6d StateDerivative(const Eigen::Vector3d& velocity,
                                       const Eigen::Vector3d& position,
                                       const double t) const;

    // operation function to integrate (function object)
    inline void operator() (const std::vector<double>& state,
                                std::vector<double>& state_derivative, const double);

};


#endif