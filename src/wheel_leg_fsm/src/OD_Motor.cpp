#include "OD_Motor.h"
#include <bitset>
void OD_motor::init(std::string side_name,uint8_t can_id,double zero_angle,double delta_angle,int turn_flag,uint8_t ack,double angle_up, double angle_low,double max_tor)
{
    this->side_name = side_name;
    this->can_id = can_id;
    this->zero_angle = zero_angle;
    this->delta_angle = delta_angle;
    this->turn_flag = turn_flag; // 0 stands for change direction
    if(ack != 1 && ack !=3){
        ROS_ERROR_STREAM(side_name<<this->can_id<<" ack model not 0x01 or 0x03 .use 0x01 .");
        this->ack_mode = 1;
    }
    else{
        this->ack_mode = ack;
    }
    this->ack_mode = ack;
    this->max_torque = max_tor;
    // std::vector<double> num = {0.57919222,0.57919222}; // 1 order Fs 400hz Fc 40Hz
    // std::vector<double> den = {1,0.15838444};
    std::vector<double> num = {0.0413,    0.0825,    0.0413}; // 2 order Fs 400hz Fc 30Hz
    std::vector<double> den = {1.0000,   -1.3490,    0.5140};
    this->speed_filter.init(2,den,num);
    if(angle_low >= angle_up){
        // no angle limit
        no_angle_limit = true;
        ROS_INFO_STREAM(side_name<<this->can_id<<" motor angle limit: "<< "None"
        <<" ack: "<<int(this->ack_mode)<<" max_tor: "<<this->max_torque);
    }
    else{
        no_angle_limit = false;
        this->ANGLE_LOW = angle_low;
        this->ANGLE_UP = angle_up;
        ROS_INFO_STREAM(side_name<<this->can_id<<" motor angle limit: "<<"["<<this->ANGLE_LOW<<","<<this->ANGLE_UP<<"]"
        <<" ack: "<<int(this->ack_mode)<<" max_tor: "<<this->max_torque);
    }
}
union Typeconverter{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
}typeconverter;
inline double angle_real_2_coor(double real_angle,double zero_angle,double delta_angle,int flag){
    return (2*flag -1)*(real_angle-zero_angle)+delta_angle;
}
inline double angle_coor_2_real(double coor_angle,double zero_angle,double delta_angle,int flag){
    return (2*flag-1)*(coor_angle-delta_angle) + zero_angle;
}
inline double angular_reverse(double angular,int flag){
    return (2*flag-1)*angular;
}
inline double torque_reverse(double torque,int flag){
    return (2*flag-1)*torque;
}
inline double rad_2_deg(double rad){
    return (rad / 3.1415926) * 180;
}
inline double deg_2_rad(double deg){
    return (deg / 180) * 3.1415926;
}
inline double rpm_2_radps(double rpm){
    return (rpm / 60) * 2 *3.1415926;
}
double clamp(double data,double upper,double lower)
{
    if(upper <= lower){
        ROS_ERROR_STREAM_THROTTLE(0.1,"clamp error : upper limit <= lower limit!");
        return data;
    }
    if(data > upper){
        return upper;
    }
    else if(data < lower){
        return lower;
    }
    else{
        return data;
    }
}
std::string slice_data(const uint8_t * data,uint8_t data_size,std::vector<uint8_t> slice){
    using std::bitset;
    std::string total_data;
    std::string result;
    for(int i=0;i<data_size;i++){
        total_data += bitset<8>(data[i]).to_string();
    }
    int sub_start=0;
    for(int i=0;i<slice.size();i++){
        result+=total_data.substr(sub_start,slice[i]);
        sub_start+=slice[i];
        result+="|";
    }
    if(sub_start!=data_size*8){
        return "slice error";
    }
    return result;
}
void OD_motor::update(can_msgs::Frame::ConstPtr can_frame){
    if(can_frame->is_error){
        ROS_WARN_STREAM(side_name<<" motor "<<can_id << "error can_frame received.");
        return;
    }
    if(can_id!=can_frame->id && can_id!=can_frame->id-0x204){
        ROS_WARN_STREAM(side_name<<" motor "<<can_id << "can not update using can_frame with can_id:"<<can_frame->id);
        return;
    }
    else if(can_id == can_frame->id){
        // ack mode 
        uint8_t ack_status = can_frame->data[0]>>5;
        if(ack_status == 1 && can_frame->dlc==7){
            // position is rad velocity is rad/s
            uint16_t position_int;
            uint16_t speed_int;
            uint16_t current_int;
            position_int=can_frame->data[1]<<8|can_frame->data[2];
            speed_int=(can_frame->data[3]<<4)|((can_frame->data[4] & 0xF0)>>4);
            current_int=(can_frame->data[4] & 0x0F)<<8 | can_frame->data[5];
            this->position = angle_real_2_coor(POS_MIN + position_int*(POS_MAX-POS_MIN)/(65535),zero_angle,delta_angle,turn_flag);
            this->raw_velocity = angular_reverse(SPD_MIN + speed_int*(SPD_MAX-SPD_MIN)/(4095),turn_flag);
            this->velocity = speed_filter.filtrate(angular_reverse(SPD_MIN + speed_int*(SPD_MAX-SPD_MIN)/(4095),turn_flag));
            this->current = torque_reverse(I_MIN + current_int*(I_MAX-I_MIN)/(4095),turn_flag);
            this->temperature = (can_frame->data[6]-50)/2;
        }
        else if(ack_status == 3 && can_frame->dlc==8){
            // velocity is rpm
			typeconverter.buf[0]=can_frame->data[4];
			typeconverter.buf[1]=can_frame->data[3];
			typeconverter.buf[2]=can_frame->data[2];
			typeconverter.buf[3]=can_frame->data[1];
            this->raw_velocity = rpm_2_radps(angular_reverse(typeconverter.to_float,turn_flag));
            this->velocity = speed_filter.filtrate(rpm_2_radps(angular_reverse(typeconverter.to_float,turn_flag)));
            int16_t current_int = can_frame->data[5]<<8|can_frame->data[6];
            this->current = torque_reverse(current_int / 100.0,turn_flag);
            this->temperature = (can_frame->data[7]-50)/2;
        }
        else{
            ROS_WARN_STREAM(side_name<<" motor "<<can_id << "msg not a standard 0x01 type message ");
            return;
        }
    }
    else{
        // report mode
        int16_t position_int = (int16_t)(can_frame->data[0]<<8|can_frame->data[1]); // [-180~180] deg
        int16_t speed_int = (int16_t)(can_frame->data[2]<<8|can_frame->data[3]); // rpm
        int16_t current_int = (int16_t)(can_frame->data[4]<<8|can_frame->data[5]); // A
        this->position = position_int/100.0;
        this->raw_velocity = rpm_2_radps(angular_reverse(speed_int/10.0,turn_flag));
        this->velocity = speed_filter.filtrate(rpm_2_radps(angular_reverse(speed_int/10.0,turn_flag)));
        this->current = torque_reverse(current_int/100.0,turn_flag);
        this->temperature = (can_frame->data[6] - 50) / 2; 
        this->error_flag = can_frame->data[7];
    }
}
can_msgs::to_ros OD_motor::get_ros_msg(){
    can_msgs::to_ros msg;
    msg.current = this->current;
    msg.motor_id = this->can_id;
    msg.position = this->position;
    msg.speed = this->velocity;
    msg.raw_speed = this->raw_velocity;
    msg.temperature = this->temperature;
    msg.torque_cmd = this->torque_cmd;
    return msg;
}
can_msgs::Frame OD_motor::set_tor(double tor){
    can_msgs::Frame can_frame;
    can_frame.id = this->can_id;
    can_frame.dlc = 3;
    can_frame.is_error = false;
    can_frame.is_extended = false;
    can_frame.is_rtr = false;

    uint8_t ctrl_status = 0x01; // use torque control not current
    uint8_t ack_status = this->ack_mode; //return type message
    tor = clamp(tor,this->max_torque,-this->max_torque);
    this->torque_cmd = tor;
    int16_t cur_int = int16_t(torque_reverse(tor *100.0,turn_flag));
    can_frame.data[0] = 0x60|(ctrl_status<<2)|ack_status;
    can_frame.data[1] = cur_int>>8; 
    can_frame.data[2] = cur_int&0xFF;

    std::vector<uint8_t> slice = {3,3,2,16};
    ROS_DEBUG_STREAM_THROTTLE(0.1,side_name<<can_id<<" torque cmd:"<<slice_data(can_frame.data.c_array(),3,slice));
    return can_frame;
}
/*
can_msgs::Frame OD_motor::set_pos(double pos,double vel,double max_cur){
    can_msgs::Frame can_frame;
    can_frame.id = this->can_id;
    can_frame.dlc = 8;
    can_frame.is_error = false;
    can_frame.is_extended = false;
    can_frame.is_rtr = false;

    uint8_t ack_status = 0x01;
    uint16_t speed = abs(10*vel);
    uint16_t cur = abs(10*max_cur);
    double pos_limit;
    if(no_angle_limit){
        pos_limit = pos;
    }
    else{
        pos_limit = clamp(pos,ANGLE_UP,ANGLE_LOW);
    }
    typeconverter.to_float = angle_coor_2_real(pos_limit,zero_angle,delta_angle,turn_flag);
    can_frame.data[0]= 0x20|(typeconverter.buf[3]>>3);
    can_frame.data[1]= (typeconverter.buf[3]<<5)|(typeconverter.buf[2]>>3);
    can_frame.data[2]= (typeconverter.buf[2]<<5)|(typeconverter.buf[1]>>3);
    can_frame.data[3]= (typeconverter.buf[1]<<5)|(typeconverter.buf[0]>>3);
    can_frame.data[4]= (typeconverter.buf[0]<<5)|(speed>>10);
    can_frame.data[5]= (speed>>2) & 0xff;
    can_frame.data[6]= (speed&0x03)<<6|cur>>6;
    can_frame.data[7]= (cur&0x3F)<<2|ack_status;
    std::vector<uint8_t> slice = {3,32,15,12,2};
    ROS_DEBUG_STREAM_THROTTLE(0.1,side_name<<can_id<<" position cmd:"<<slice_data(can_frame.data.c_array(),8,slice));    
    return can_frame;
}
can_msgs::Frame OD_motor::set_vel(double vel,double max_cur){
    can_msgs::Frame can_frame;
    can_frame.id = this->can_id;
    can_frame.dlc = 7;
    can_frame.is_error = false;
    can_frame.is_extended = false;
    can_frame.is_rtr = false;

    uint8_t ack_status = 0x01; 
    uint16_t cur = abs(max_cur*10);    
    typeconverter.to_float = angular_reverse(vel,turn_flag);
    can_frame.data[0]=0x40|ack_status;
    can_frame.data[1]=typeconverter.buf[3];
    can_frame.data[2]=typeconverter.buf[2];
    can_frame.data[3]=typeconverter.buf[1];
    can_frame.data[4]=typeconverter.buf[0];
    can_frame.data[5]=cur>>8;
    can_frame.data[6]=cur&0xff; 
    std::vector<uint8_t> slice = {3,3,2,32,16};
    ROS_DEBUG_STREAM_THROTTLE(0.1,side_name<<can_id<<" speed cmd:"<<slice_data(can_frame.data.c_array(),7,slice));      
    return can_frame;
}
can_msgs::Frame OD_motor::set_tor(double pos,double vel,double forward_tor,double Kp,double Kd,double max_tor){
    can_msgs::Frame can_frame;
    can_frame.id = this->can_id;
    can_frame.dlc = 3;
    can_frame.is_error = false;
    can_frame.is_extended = false;
    can_frame.is_rtr = false;

    uint8_t ctrl_status = 0x01; // use torque control not current
    uint8_t ack_status = 0x01; //return type 1 message
    double pos_limit;
    if(no_angle_limit){
        pos_limit = pos;
    }
    else{
        pos_limit = clamp(pos,ANGLE_UP,ANGLE_LOW);
    }
    double tor = forward_tor + Kp * (pos_limit-this->position) + Kd * (vel-this->velocity);
    tor = clamp(tor,max_tor,-max_tor);
    this->torque_cmd = tor;
    int16_t cur_int = int16_t(torque_reverse(tor *100.0,turn_flag));
    can_frame.data[0] = 0x60|(ctrl_status<<2)|ack_status;
    can_frame.data[1] = cur_int>>8; 
    can_frame.data[2] = cur_int&0xFF;

    std::vector<uint8_t> slice = {3,3,2,16};
    ROS_DEBUG_STREAM_THROTTLE(0.1,side_name<<can_id<<" torque cmd:"<<slice_data(can_frame.data.c_array(),3,slice));
    return can_frame;
}
*/
can_msgs::Frame OD_motor::set_mix(double pos,double vel,double forward_tor,double Kp,double Kd){
    can_msgs::Frame can_frame;
    can_frame.id = this->can_id;
    can_frame.dlc = 8;
    can_frame.is_error = false;
    can_frame.is_extended = false;
    can_frame.is_rtr = false;

    uint16_t kp=uint16_t((Kp / KP_MAX)*4095);//kp range[0,4095]
    uint16_t kd=uint16_t((Kd / KD_MAX)*511);//kd range[0,511]
    double pos_limit;
    if(no_angle_limit){
        pos_limit = pos;
    }
    else{
        pos_limit = clamp(pos,ANGLE_UP,ANGLE_LOW);
    }

    uint16_t des_position = uint16_t((angle_coor_2_real(pos_limit,zero_angle,delta_angle,turn_flag)-POS_MIN)/(POS_MAX-POS_MIN)*65535);
    uint16_t des_speed = uint16_t((angular_reverse(vel,turn_flag)-SPD_MIN)/(SPD_MAX - SPD_MIN)*4095);
    uint16_t des_torque = uint16_t((torque_reverse(forward_tor,turn_flag)-T_MIN)/(T_MAX-T_MIN)*4095);

    can_frame.data[0]=0x00|(kp>>7);//kp前5位
    can_frame.data[1]=((kp & 0x7F)<<1)|((kd & 0x100)>>8);//kp后7位—+kd第一位
    can_frame.data[2]=kd & 0xFF;
    can_frame.data[3]=des_position>>8;
    can_frame.data[4]=des_position & 0xFF;
    can_frame.data[5]=des_speed>>4;
    can_frame.data[6]=((des_speed & 0x0F)<<4)|(des_torque>>8);
    can_frame.data[7]=des_torque & 0xFF;
    std::vector<uint8_t> slice = {3,12,9,16,12,12};
    ROS_DEBUG_STREAM_THROTTLE(0.1,side_name<<can_id<<" mix cmd:"<<slice_data(can_frame.data.c_array(),8,slice));
    return can_frame;
}

OD_motor::~OD_motor()
{
}
OD_motor::OD_motor()
{
    this->position = 0;
    this->velocity = 0;
    this->current = 0;
    this->temperature = -1;
    this->raw_velocity = 0;
    this->torque_cmd = 0;
    this->error_flag = 0;
}
