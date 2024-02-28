#pragma once
#include "sbus_serial/Sbus.h"
#include "OD_Param.h"
#include <eigen3/Eigen/Dense>
class Ratio
{
private:
    enum RC_channel_t{
        /* simulation robot radio RT-9s*/
        // LV=2,//left vertical
        // LH=3,//left horizontal
        // RV=1,//right vertical
        // RH=0,//right horizontal
        // AA=9,//A channel
        // BB=8,//B channel
        // CC=4,//C channel
        // DD=5, //D channel
        // LK=6,// Left knob
        // RK=7 //right knob

        /*  real radio FS-i6X*/
        LV=2,//left vertical
        LH=3,//left horizontal
        RV=1,//right vertical
        RH=0,//right horizontal
        AA=6,//A channel
        BB=7,//B channel
        CC=8,//C channel
        DD=9, //D channel
        LK=4,// Left knob
        RK=5 //right knob
    };
    WLParam param;
    double raw_channel_transform(double value,double mid,double upper,double lower);
public:

    int A,B,C,D; // discrate -1 0 1
    double left_v,left_h,right_v,right_h,left_knob,right_knob,dx,dw; // continues [-1,1]
    Ratio(WLParam &param_);
    void update(sbus_serial::SbusConstPtr sbus_msg);
    ~Ratio();
};


