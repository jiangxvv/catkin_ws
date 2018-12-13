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

#ifndef THRUST_ALLOCATION_H
#define THRUST_ALLOCATION_H

#include <array>
#include <sstream>
#include "usv_sim/control.h"
#include "Array.h"

typedef struct
{

  double T_fwd;
  double T_rev;
  double lx;
  double ly;

} thrust_allocaton_params;

class ThrustAllocation
{
public:
  /////////////////
  // Constructor //
  /////////////////

  ThrustAllocation() = default;
  ThrustAllocation(const double& T_fwd, const double& T_rev, const double& lx,
                   const double& ly);

  ////////////////////
  // Public Methods //
  ////////////////////

  std::array<double, 7> ComputeOutput(const std::array<double, 3>& input);

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // maximum actuation when forward
  double T_fwd_ = 0;

  // maximum actuation when reverse
  double T_rev_ = 0;

  // the coordinate of the azimuth thruster in the body-fixed frame
  double lx_ = 0;
  double ly_ = 0;

  // variables for QuadProg++
  Matrix<double> G_, CE_;
  Matrix<double> CI_1_, CI_2_, CI_3_, CI_4_;
  Vector<double> g0_, x_;
  Vector<double> ce0_;
  Vector<double> ci0_1_, ci0_2_, ci0_3_, ci0_4_;
  int n_, m_, p_;
};

#endif // THRUST_ALLOCATION_H
