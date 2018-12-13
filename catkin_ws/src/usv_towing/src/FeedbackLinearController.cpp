// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class for feedback linear controller
#include<Eigen/Dense>
#include"BasicController.h"
#include"FeedbackLinearController.h"

FeedbackLinearController::FeedbackLinearController(const VesselControlProperty &vessel, 
                                                   const doubel &step_size, const double &kp, const double &k)
       :BasicController(vessel, step_size), kp_(kp), k_(k)
{
    
}




void FeedbackLinearController::ComputeControl()
{
    //the parameters of the equation, M, C, D, here D is a vector.
    Eigen::Matrix3d M=BasicController::InertiaMatrix();
    Eigen::Matrix3d C=BasicController::InertiaCCMatrix(vessel_velocity_);
    Eigen::Vector3d D=BasicController::DampingVector(vessel_velocity_);
    
    double r=vessel_velocity_(2); // yaw vessel_velocity_
    Eigen::Matrix3d  S<< 0, -r, 0, r, 0, 0, 0, 0, 0;
    
    // rotation matrix
    Eigen::Matrix3d R=BasicController::RotationMatrix(vessel_position_);
    Eigen::Matrix3d Rt=R.transpose();
    
    error_pose_=Rt*(vessel_position_-course_position_);
    error_vel_=Rt*error_pose_-vessel_velocity_;
    Eigen::Vector3d error=error_vel_+kp_*error_pose_;
    
    
    output_=C*vessel_velocity_+D+error_pose_+k_*error
            +M*(-S*(Rt*course_position_+kp_*error_pose_)+Rt*course_velocity
            +kp_error_vel_);
                  
}

