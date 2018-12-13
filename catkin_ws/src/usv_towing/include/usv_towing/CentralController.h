// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class of the central controller

#ifndef CENTRALCONTROLLER_H
#define CENTRALCONTROLLER_H

#include<ros/ros.h>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>
#include "usv_towing/control.h"
#include"usv_towing/course.h"
#include"MotherShip.h"

// forward declare
class MotheShip;

// class define
class CentralController
{
	
public:
// constructor
	CentralController(const double& time_step,const double& kp, const double& k);

	// public methods
	int run()
	{
		ros::spin();
		return EXIT_FAILURE;
	}
	
	virtual ~CentralController();

private:
	// private field
	// ros node
	ros::NodeHandle nh_;

	// publisher and message of the actuation signal
	std::unique_ptr<ros::Publisher> pubPtr_control_;
	usv_towing::control msg_actuation_;

	// create a timer to control the publishing of control commands
	ros::Timer timer_;

	// time step 
	double time_step_;

	// subscribers
	std::unique_ptr<ros::Subscriber> sub_pos_;
	std::unique_ptr<ros::Subscriber> sub_vel_;
	std::unique_ptr<ros::Subscriber> sub_course_;

	// mothership class
	std::unique_ptr<MotherShip> Ptr_vehicle_;
	
	// actuation computed by the controller
	Eigen::Vector3d actuation_;

	// helper variable
	bool received_course_info_=false;
	
	// parameters of controller
	double kp_, k_;

	// variables used for the controller
	double u_, v_, r_;
	double x_, y_, psi_;

	// position and velocity of the vehicle
	Eigen::Vector3d position_, velocity_;
	
	// course variables
	Eigen::Vector3d pd_, pd1_, pd2_;

	// errors of the motion
	Eigen::Vector3d error_pose_, error_vel_, error_;

	// dynamic matrix
	Eigen::Matrix3d mass_, CCmatrix_;
	Eigen::Vector3d damping_;

	// rotation matrix
	Eigen::Matrix3d R_, Rt_;

	// skew-symmetric matrix
	Eigen::Matrix3d SS_;

	// private method
	void callback_pose(const geometry_msgs::Pose2D& msg_pose)
	{
		x_=msg_pose.x;
		y_=msg_pose.y;
		psi_=msg_pose.theta;

		position_<<x_, y_, psi_;
	}

	void callback_vel(const geometry_msgs::Twist& msg_vel)
  	{
            u_ = msg_vel.linear.x;
            v_ = msg_vel.linear.y;
            r_ = msg_vel.angular.z;
		
            velocity_<<u_, v_, r_;
  	}

 	void callback_course(const usv_towing::course& msg_course)
 	{
    	pd_<< msg_course.pose.x, msg_course.pose.y, msg_course.pose.z;
    	pd1_<<msg_course.velocity.x, msg_course.velocity.y, msg_course.velocity.z;
   		pd2_<<msg_course.acceleration.x, msg_course.acceleration.y, msg_course.acceleration.z;

    	received_course_info_ = true;
 	}	  

	void timerCallback(const ros::TimerEvent&);
	void get_ros_param();

public:
	Eigen::Vector3d ActuationCompute();

};

#endif