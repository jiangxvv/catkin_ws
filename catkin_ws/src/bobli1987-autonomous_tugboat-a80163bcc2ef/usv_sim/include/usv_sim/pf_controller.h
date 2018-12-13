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

#ifndef PF_CONTROLLER_H
#define PF_CONTROLLER_H

#include <cmath>
#include <Eigen/Dense>

/////////////
/// The nonlinear, set-point controller for path-following missions
/// For details, see Ivan's paper published in Ocean Engineering 106 (2015):
/// 496-514
///

namespace path_following
{

typedef struct
{

  double ku;        // surge speed controller gain
  double k1;        // heading controller parameter 1
  double k2;        // heading controller parameter 2
  double u_acc_max; // the maximum acceleration of the vehicle
  double k_acc_max; // the slope
  double u_d_yaw;   // the maximum velocity during turns

} control_params;

typedef struct
{

  Eigen::Vector3d M;
  Eigen::Vector3d D;

} vehicle_params;

class PFController
{

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // controller parameters
  double ku_;
  double k1_;
  double k2_;
  double u_acc_max_;
  double k_acc_max_;
  double u_d_yaw_;

  // coefficients of the physical model
  double Mu_; // mass plus added mass
  double Mv_;
  double Mr_;
  double Xu_; // linear damping coefficient
  double Yv_;
  double Nr_;

public:
  /////////////////
  // Constructor //
  /////////////////

  PFController() = default;
  PFController(const vehicle_params& v_param, const control_params& c_param)
  {
    ku_ = c_param.ku;
    k1_ = c_param.k1;
    k2_ = c_param.k2;
    u_acc_max_ = c_param.u_acc_max;
    k_acc_max_ = c_param.k_acc_max;
    u_d_yaw_ = c_param.u_d_yaw;

    Mu_ = v_param.M(0);
    Mv_ = v_param.M(1);
    Mr_ = v_param.M(2);
    Xu_ = v_param.D(0);
    Yv_ = v_param.D(1);
    Nr_ = v_param.D(2);
  }

  ///////////////////
  // Public Method //
  ///////////////////

  // the member method used to compute actuation
  Eigen::Vector3d ComputeActuation(const Eigen::Vector3d& vel,
                                   const Eigen::Vector3d& pos,
                                   const double& u_d, const double& psi_d)
  {
    Eigen::Vector3d output;

    double u = vel[0];
    double v = vel[1];
    double r = vel[2];
    double psi = pos[2];

    double u_d_ref = std::min(
        u_d_yaw_ + (u_d - u_d_yaw_) * std::exp(-5.73 * std::fabs(psi - psi_d)),
        u_d);
    double u_d_dot =
        u_acc_max_ * std::tanh(k_acc_max_ * (u_d_ref - u) / u_acc_max_);

    output[0] = Mu_ * (u_d_dot - ku_ * (u - u_d_ref)) - (Mv_ * v * r + Xu_ * u);
    output[1] = 0;
    output[2] =
        Mr_ * (-k1_ * (psi - psi_d) - k2_ * r) - (Mu_ - Mv_) * u * v - Nr_ * r;

    return output;
  }
};
}

#endif // PF_CONTROLLER_H
