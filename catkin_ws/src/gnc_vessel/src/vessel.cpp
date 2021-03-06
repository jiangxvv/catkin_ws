// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class of vessel

#include"gnc_vessel/vessel.h"
#include<boost/numeric/odeint.hpp>
#include<cmath>

Vessel::Vessel(const VesselParameters &config, const std::vector< double > &init_velocity,                     const std::vector< double > &init_position, const std::vector< double > &init_actuation)
    :init_velocity_(init_velocity), init_position_(init_position), init_actuation_(init_actuation)
{
    velocity_ << init_velocity[0], init_velocity[1], init_velocity[2];
    position_ << init_position[0], init_position[1], init_position[2];
    actuation_ << init_actuation[0], init_actuation[1], init_actuation[2];
    
    velocity_history_ = {velocity_};
    position_history_ = {position_};
    
    name_=config.name;
    mass_=config.mass;
    radius_=config.radius;
    Izz_=mass_*radius_*radius_;
    length_=config.length;
    a11_=config.a11;
    a22_=config.a22;
    a23_=config.a23;
    a33_=config.a33;
    Xu_=config.Xu;
    Yv_=config.Yv;
    Nr_=config.Nr;
   
}



// rotation matrix 
inline Eigen::Matrix3d Vessel::RotationMatrix(const Eigen::Vector3d& pose) const
{
    Eigen::Matrix3d rz;
    rz<< cos(pose[2]),-sin(pose[2]), 0, sin(pose[2]), cos(pose[2]), 0, 0, 0, 1;
    return rz;
}

// inertia matrix of the vehicle
inline Eigen::Matrix3d Vessel::InertiaMatrix() const
{
  Eigen::Matrix3d inertia_mat;
  inertia_mat << mass_ + a11_, 0, 0, 0, mass_ + a22_, a23_, 0, a23_,
      Izz_ + a33_;
  return inertia_mat;
}

// inertial centripetal and coriolis matrix
inline Eigen::Matrix3d Vessel::InertiaCCMatrix(const Eigen::Vector3d& vec) const
{
  Eigen::Matrix3d inertia_cc_mat;
  inertia_cc_mat << 0, -mass_* vec[2], -a23_* vec[2] - a22_* vec[1],
      mass_* vec[2], 0, a11_* vec[0], a23_* vec[2] + a22_* vec[1],
      -a11_* vec[0], 0;
  return inertia_cc_mat;
}

// damping matrix of the vehicle  !!!(need to be classified later)
Eigen::Vector3d Vessel::DampingVector(const Eigen::Vector3d& vec) const
{
  double U =
      std::sqrt(std::pow(vec[0], 2) + std::pow(vec[1], 2)); // vehicle speed
  double Xu=Xu_*rou_/2*pow(length_,2)*U;
  double Yv=Yv_*rou_/2*pow(length_,2)*U;
  double Nr=Nr_*rou_/2*pow(length_,4)*U; // transfer the nondimension to the real one
  //double Nr =
  //   (U > 0.3) ? (600 * U + 200) : 400; // Nr depends on the vehicle speed
//   double Xu=Xu_*  
  Eigen::Vector3d damping1, damping2;
  damping1 << Xu* vec[0], Yv* vec[1],  Nr* vec[2];

  // Xu|u| is different in forward and reverse conditions
  //double Xuu = (vec[0] > 0) ? 112.4 : 198.1;
  //double Xuu=0;
  //damping2 << Xuu* vec[0] * std::fabs(vec[0]), 0, 0;
  return damping1 ;
}

// time derivative of the system state
Vessel::Vector6d Vessel::StateDerivative(const Eigen::Vector3d& velocity,
                                       const Eigen::Vector3d& position,
                                       const double t) const
{
    Vessel::Vector6d state_derivative;
    Eigen::Vector3d vec1,vec2;

    vec1=InertiaMatrix().inverse() * (actuation_ - DampingVector(velocity) -
                                      InertiaCCMatrix(velocity) * velocity);
    vec2=RotationMatrix(position)*velocity;
    state_derivative<<vec1,vec2;
    return state_derivative;
}

void Vessel::operator() (const std::vector<double>& state,
                        std::vector<double>& state_derivative, const double t)
{
    Eigen::Vector3d velocity, position;
    Vessel::Vector6d dy;
    velocity<<state[0],state[1],state[2];
    position<<state[3],state[4],state[5];
    dy=StateDerivative(velocity,position,t);
    for (size_t index=0; index<6; ++index)
        state_derivative[index]=dy[index];
}

// conduct a simulation of the vehicle's motion
void RunVessel(Vessel& vehicle, const size_t& step_number=600,
                const double& step_size=0.1)
{
    std::cout<<"RunVessel is called"<<std::endl;
    vehicle.step_number_=step_number;
    vehicle.step_size_=step_size;

    // ode solve
    boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper;

    std::vector<double> state;

    for (size_t index=0; index<3; ++index)
        state.push_back(vehicle.velocity_[index]);
    for (size_t index=0; index<3;++index)
        state.push_back(vehicle.position_[index]);
    for (size_t counter=1; counter<=step_number; ++counter)
    {
        stepper.do_step(vehicle, state, vehicle.current_time_, step_size);

        // update the data members of the object
        ++vehicle.step_counter_;
        vehicle.current_time_+=step_size;
        vehicle.time_vec_.push_back(vehicle.current_time_);
        vehicle.velocity_<<state[0], state[1], state[2];
        vehicle.position_ << state[3], state[4], state[5];

        // wrap the heading angle
        vehicle.position_(2)=remainder(vehicle.position_[2],2*M_PI);
        vehicle.velocity_history_.push_back(vehicle.velocity_);
        vehicle.position_history_.push_back(vehicle.position_);
    }
}




    
    
    
    