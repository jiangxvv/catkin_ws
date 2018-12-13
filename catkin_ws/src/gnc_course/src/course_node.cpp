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

#include<Eigen/Dense>
#include<ros/ros.h>
#include"basic_course.h"
#include"straight_line.h"
#include"course_interface.h"

CourseInterface::CourseInterface(const double& time_step,
                                 const Eigen::Vector3d& accel,
                                 const Eigen::Vector3d& init_vel,
                                 const Eigen::Vector3d& init_pose)
    :time_step_(time_step), accel_(accel), 
     init_vel_(init_vel), init_pose_(init_pose)
{
    
    timer_=nh_.createTimer(ros::Duration(time_step_),
                           &CourseInterface::timerCallback, this);
    
    pubPtr_course_=std::unique_ptr<ros::Publisher>(new ros::Publisher(
        nh_.advertise<gnc_msgs::course>("mothership/course", 1000)));
    
    Ptr_StraightLine_=std::unique_ptr<StraightLine>(new StraightLine(
        time_step_, accel_, init_vel_, init_pose_));
}

void CourseInterface::timerCallback(const ros::TimerEvent&)
{
    Ptr_StraightLine_->computeStraightLine();
    Eigen::Vector3d pd, pd1, pd2;
    pd=Ptr_StraightLine_->getPd();
    pd1=Ptr_StraightLine_->getPd1();
    pd2=Ptr_StraightLine_->getPd2();
    
    std::cout<<pd<<std::endl;
    
    msg_course_.pose.x=pd[0]; msg_course_.pose.y=pd[1], msg_course_.pose.z=pd[2];
    msg_course_.velocity.x=pd1[0];
    msg_course_.velocity.y=pd1[1];
    msg_course_.velocity.z=pd1[2];
    msg_course_.acceleration.x=pd2[0];
    msg_course_.acceleration.y=pd2[1];
    msg_course_.acceleration.z=pd2[2];
    
    pubPtr_course_->publish(msg_course_);  
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "course_node");
    
    double time_step=0.05;
    Eigen::Vector3d accel, init_vel, init_pose;
    accel<<0.05, 0, 0;
    init_vel<< 0, 0, 0;
    init_pose<< 0.1, 0, 0;
    
    CourseInterface straight_line(time_step, accel, init_vel, init_pose);
    ros::spin();
    return 0;
}