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

#ifndef COURSE_INTERFACE_H
#define COURSE_INTERFACE_H


#include<Eigen/Dense>
#include<memory>
#include<ros/ros.h>
#include"gnc_msgs/course.h"

#include"straight_line.h"
#include"basic_course.h"

class CourseInterface 
{
public:
    CourseInterface(const double& time_step,
                    const Eigen::Vector3d& accel, 
                    const Eigen::Vector3d& init_vel, 
                    const Eigen::Vector3d& init_pose);
    
private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    
    double time_step_;
    Eigen::Vector3d accel_;
    Eigen::Vector3d init_vel_;
    Eigen::Vector3d init_pose_;
    
    std::unique_ptr<StraightLine> Ptr_StraightLine_;
    
    std::unique_ptr<ros::Publisher> pubPtr_course_;
    gnc_msgs::course msg_course_;
    
    void timerCallback(const ros::TimerEvent&);
    
};



#endif