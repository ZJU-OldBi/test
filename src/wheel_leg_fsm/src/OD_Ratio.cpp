#include "OD_Ratio.h"

double clamp(double data,double upper,double lower);
double dead_zone(double data,double zone){
    return data>zone?(data-zone):(data<-zone?(data+zone):0);
}
Ratio::Ratio(WLParam &param_) : param(param_)
{
    A = -1;
    B = -1;
    C = -1;
    D = -1;
    left_v = 0.0;
    left_h = 0.0;
    left_knob = 0.0;
    right_v = 0.0;
    right_h = 0.0;
    right_knob = 0.0;
}
void Ratio::update(sbus_serial::SbusConstPtr sbus_msg){
    right_v = dead_zone(clamp((sbus_msg->mappedChannels[RC_channel_t::RV] - 500.0) / 500, 1.0 , -1.0), 0.1);
    right_h = dead_zone(clamp((sbus_msg->mappedChannels[RC_channel_t::RH] - 500.0) / 500, 1.0 , -1.0), 0.1);
    right_knob = dead_zone(clamp((sbus_msg->mappedChannels[RC_channel_t::RK] - 500.0) / 500, 1.0 , -1.0), 0.1);

    left_v = dead_zone(clamp((sbus_msg->mappedChannels[RC_channel_t::LV] - 500.0) / 500, 1.0 , -1.0), 0.1);
    left_h = dead_zone(clamp((sbus_msg->mappedChannels[RC_channel_t::LH] - 500.0) / 500, 1.0 , -1.0), 0.1);
    // left_h = raw_channel_transform(sbus_msg->rawChannels[RC_channel_t::LH],981,1278,618); // use raw channels
    left_knob = dead_zone(clamp((sbus_msg->mappedChannels[RC_channel_t::LK] - 500.0) / 500, 1.0 , -1.0), 0.1);

    if(sbus_msg->mappedChannels[RC_channel_t::AA]>0.5*(param.sw_low+param.sw_high)){
        A = 1;
    }
    else{
        A = -1;
    }

    if(sbus_msg->mappedChannels[RC_channel_t::BB]>0.5*(param.sw_low+param.sw_high)){
        B = 1;
    }
    else{
        B = -1;
    }

    if(sbus_msg->mappedChannels[RC_channel_t::CC]>param.sw_high){
        C = 1;
    }
    else if(sbus_msg->mappedChannels[RC_channel_t::CC]>param.sw_low){
        C = 0;
    }
    else{
        C = -1;
    }  

    if(sbus_msg->mappedChannels[RC_channel_t::DD]>0.5*(param.sw_low+param.sw_high)){
        D = 1;
    }
    else{
        D = -1;
    }  
}
double Ratio::raw_channel_transform(double value,double mid,double upper,double lower){
    if(value > mid){
        return dead_zone(clamp((value-mid)/(upper-mid),1.0,0.0),0.1);
    }
    else{
        return dead_zone(clamp(-(value-mid)/(lower-mid),0.0,-1.0),0.1);
    }
}
Ratio::~Ratio()
{
}