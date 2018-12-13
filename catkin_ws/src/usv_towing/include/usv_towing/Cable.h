// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class to transmit the force to position 

#ifndef CABLE_H
#define CABLE_H

#include<iostream>
#include<Eigen/Dense>
#include<fstream>
#include<string>
#include<memory>
#include<boost/math/interpolators/barycentric_rational.hpp>
// boost::math::tools::barycentric_rational<double> interpolant(x.data(), y.data(), y.sizie(),5)



class Cable 
{
public:
    
    // constructor
    Cable(const double& length);
    
    //destructor
    ~Cable();
    
    // private field and method
private:
    double length_;
    double force_;
    
    std::vector<double> storingforce_;
    std::vector<double> storinglength_;

    
    // method reading the relation between force and length
    void readStoringData();
    
    // private method which computes the stretch length through the force 
    double computeLength(const double& force);
};









#endif