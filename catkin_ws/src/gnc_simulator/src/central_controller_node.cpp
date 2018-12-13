/*
 * 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Xu Jiang <jiangxvv@sjtu.edu.cn>
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
 *   * Neither the name of Xu Jiang nor the names of its contributors may
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


#include<ros/ros.h>
#include"central_controller_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "central_controller");
    
    VesselControlProperty config;
    config.name="mothership";
    config.mass=33400000;
    config.radius=33.54;
    config.length=95;
    config.a11=1.2E7;
    config.a22=3.3E7;
    config.a23=0;
    config.a33=2.8E10;
    config.xg=0;
    config.d11=0.8;
    config.d22=1.7;
    config.d33=0;
    
    double time_step=0.05;
    double kp=5.0;
    double k=1.0;
    
    double cable_length=400;
    Eigen::Vector3d touch_position1, touch_position2;
    touch_position1<<config.length/2, 32.5, 0;
    touch_position2<<config.length/2, -32.5, 0;
    
    CentralControllerInterface CentralController(config, time_step, kp, k, cable_length,
                                       touch_position1, touch_position2);
    
    
    return CentralController.run();
    
}