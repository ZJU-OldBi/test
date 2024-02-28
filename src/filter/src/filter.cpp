#include <ros/ros.h>
#include "../include/filter.h"
namespace Filter
{
    lpf::lpf()
    {
    }
    
    lpf::~lpf(){};
    void lpf::init(int order,std::vector<double> den_,std::vector<double> num_){
        this->order = order;
        this->den = den_;
        this->num = num_;
        if((order+1)!=den_.size() || (order+1)!=num_.size()){
            ROS_ERROR_STREAM("order+1 != den_size: "<<(order+1)<<"!="<<den_.size());
        }
        // x_n-1  ->  x_n-order
        for(int i=0;i<order;i++){
            x_buffer.push_back(0.0);
            y_buffer.push_back(0.0);
        }
    }
    double lpf::filtrate(double value){
        double y_n = value * num[0];
        for(int i=0;i<order;i++){
            y_n +=  x_buffer[i] *num[i+1];
        }
        for(int i=0;i<order;i++){
            y_n -=  y_buffer[i] *den[i+1];
        }
        x_buffer.insert(x_buffer.begin(),value);
        x_buffer.pop_back();
        y_buffer.insert(y_buffer.begin(),y_n);
        y_buffer.pop_back();

        return y_n;
    }
}