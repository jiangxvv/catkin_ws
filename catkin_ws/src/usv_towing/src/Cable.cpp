// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class to transmit the force to position

#include"Cable.h"

#include<iostream>
#include<Eigen/Dense>
#include<fstream>
#include<sstream>
#include<string>
#include<memory>
//#include<boost/math/interpolators/barycentric_rational.hpp>

#include<gsl/gsl_errno.h>
#include<gsl/gsl_spline.h>

Cable::Cable(const double &length) :length_(length) 
{
    readStoringData();
    
}


void Cable::readStoringData()
{
    std::fstream file;
    file.open("home/jiangxvv/catkin_ws/src/usv_towing/cfg/cable.txt");
    if(file.fail())
    {
        std::cout<< "cable file reads fail!"<<std::endl;
    }
            
    
    // reading matrix data from txt
    string str;
    std::vector<std::vector<double>> num;
    
    while(std::getline(file, str))
    {
        std::istringstream input(str);
        std::vector<double> tmp;
        double a;
        while(input>>a)
            tmp.push_back(a);
        num.push_back(tmp);
        
    }
    
    for(int i=0; i<num.size; ++i)
    {
        storingforce_.push_back(num[i][0]);
        storinglength_.push_back(num[i][1]);
        
    }
    
}



double Cable::computeLength(const double &force)
{
    
    //boost::math::barycentric_rational<double> interpolator(
       // storingforce_.data(), storinglength_.data(), storingforce_.size());
    
    unsigned int data_size=storingforce_.size();
    
    acc_=gsl_interp_accel_alloc();
    spline_=gsl_spline_alloc(gsl_interp_cspline,data_size);
    
    gsl_spline_init(spline_, storingforce_, storinglength_, data_size);
    
    double length= gsl_spline_eval(spline_, force, acc_);
    
    gsl_spline_free(spline_);
    gsl_interp_accel_free(acc_);
    
    return length;
}


























