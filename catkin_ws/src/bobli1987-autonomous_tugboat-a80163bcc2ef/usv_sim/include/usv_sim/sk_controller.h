/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bo Li <lither@sjtu.edu.cn>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bo Li nor the names of its contributors may
 *     be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SK_CONTROLLER_H
#define SK_CONTROLLER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

// the parameters of station-keeping controller
namespace station_keeping
{

typedef struct
{

  Eigen::Vector3d kp;
  Eigen::Vector3d kd;
  Eigen::Vector3d nl_scale;
  Eigen::Vector3d lambda;

} control_params;

typedef struct
{

  Eigen::Vector3d M;
  Eigen::Vector3d D;

} vehicle_params;

class SKController
{

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // controller parameters
  Eigen::Matrix3d kp_mat_;
  Eigen::Matrix3d kd_mat_;
  Eigen::Matrix3d scale_mat_;
  Eigen::Matrix3d lambda_mat_;

  // matrices of the physical model
  Eigen::Matrix3d M_;
  Eigen::Matrix3d C_;
  Eigen::Matrix3d D_;

  ////////////////////
  // Private Method //
  ////////////////////d

  // inline member methods
  inline Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d&);
  inline Eigen::Matrix3d RotationMatrix_Inverse(const Eigen::Vector3d&);
  inline Eigen::Matrix3d RotationMatrix_Inverse_dot(const Eigen::Vector3d& pos,
                                                    const Eigen::Vector3d& vel);

public:
  /////////////////
  // Constructor //
  /////////////////

  SKController() = default;
  SKController(const vehicle_params& v_param, const control_params& c_param)
  {
    kp_mat_ = c_param.kp.asDiagonal();
    kd_mat_ = c_param.kd.asDiagonal();
    scale_mat_ = c_param.nl_scale.asDiagonal();
    lambda_mat_ = c_param.lambda.asDiagonal();

    M_ = v_param.M.asDiagonal();
    D_ = v_param.D.asDiagonal();
  }

  ///////////////////
  // Public Method //
  ///////////////////

  // the member method used to compute actuation
  Eigen::Vector3d ComputeActuation(const Eigen::Vector3d& pos,
                                   const Eigen::Vector3d& vel,
                                   const Eigen::Vector3d& pos_ref,
                                   const Eigen::Vector3d& pos_ref_dot,
                                   const Eigen::Vector3d& pos_ref_ddot)
  {
    Eigen::Vector3d pos_error = pos - pos_ref;

    // calculate vr and vr_dot
    Eigen::Vector3d vr =
        RotationMatrix_Inverse(pos) * (pos_ref_dot - lambda_mat_ * pos_error);
    Eigen::Vector3d vr_dot =
        RotationMatrix_Inverse(pos) *
            (pos_ref_ddot -
             lambda_mat_ * (RotationMatrix(pos) * vel - pos_ref_dot)) -
        RotationMatrix_Inverse_dot(pos, vel) *
            (pos_ref_dot - lambda_mat_ * pos_error);

    // calculate the centripetal-coriolis matrix
    C_ << 0, 0, -M_(1, 1) * vel[1], 0, 0, M_(0, 0) * vel[0], M_(1, 1) * vel[1],
        -M_(0, 0) * vel[0], 0;

    // calculate the output
    Eigen::Vector3d s =
        RotationMatrix(pos) * vel - (pos_ref_dot - lambda_mat_ * pos_error);

    Eigen::Vector3d output = scale_mat_ * (M_ * vr_dot + C_ * vr + D_ * vr) -
                             RotationMatrix_Inverse(pos) * kd_mat_ * s -
                             RotationMatrix_Inverse(pos) * kp_mat_ * pos_error;
    return output;
  }
};

////////////////////
// Private Method //
////////////////////

Eigen::Matrix3d SKController::RotationMatrix(const Eigen::Vector3d& pose)
{
  Eigen::Matrix3d rz;
  rz << std::cos(pose[2]), -std::sin(pose[2]), 0, std::sin(pose[2]),
      std::cos(pose[2]), 0, 0, 0, 1;
  return rz;
}

Eigen::Matrix3d
SKController::RotationMatrix_Inverse(const Eigen::Vector3d& pose)
{
  Eigen::Matrix3d out;
  out << std::cos(pose[2]), std::sin(pose[2]), 0, -std::sin(pose[2]),
      std::cos(pose[2]), 0, 0, 0, 1;
  return out;
}

Eigen::Matrix3d
SKController::RotationMatrix_Inverse_dot(const Eigen::Vector3d& pose,
                                         const Eigen::Vector3d& vel)
{
  Eigen::Matrix3d out;
  out << -std::sin(pose[2]), std::cos(pose[2]), 0, -std::cos(pose[2]),
      -std::sin(pose[2]), 0, 0, 0, 0;
  return vel[2] * out;
}
}

#endif // SK_CONTROLLER_H
