// Jiang Xu
// jiangxvv@sjtu.edu.cn

// class for feedback linear controller
#ifndef FEEDBACKLINEARCONTROLLER_H
#define FEEDBACKLINEARCONTROLLER_H

#include<iostream>
#include<memory>
#include<Eigen/Dense>
#include<vector>
#include<string>
#include<cmath>

#include"BasicController.h"

// class feedback linearization controller, which is derived from BasicController

class FeedbackLinearController: public BasicController
{
private:
    // controller gains 
    double kp_, k_;
    
public:
    
    // brief a constructor
    FeedbackLinearController(const VesselControlProperty& vessel, const double& step_size, 
        const double& kp, const double& k);
   
    // brief a destructor
    ~FeedbackLinearController(){}
    
    // brief compute control output
    void ComputeControl();
    
    
    

    // return gain  kp 
    double getKpGain() const
    {
        return kp_;
    }
    
    // return gain  k 
    double getKGain() const
    {
        return k_;
    }
    
    
    // set kp gain
    void setKpGain(const double& input)
    {
        kp_=input;
    }
    
    
    // set k gain
    void setKGain(const double& input)
    {
        k_=input;
    }
    
};

    






#endif
