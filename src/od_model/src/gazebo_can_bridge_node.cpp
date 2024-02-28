#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "can_msgs/Frame.h"
#include "can_msgs/to_ros.h"
#include "std_msgs/Float64.h"

union Typeconverter{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
}typeconverter;

inline double rpm_2_radps(double rpm){
    return (rpm / 60) * 2 *3.1415926;
}
inline double radps_2_rpm(double radps){
    return (radps / 2 / 3.1415926) * 60;
}
double left_q0,left_q1,left_q2,left_q3,right_q0,right_q1,right_q2,right_q3;
double left_t0,left_t1,left_t2,left_t3,right_t0,right_t1,right_t2,right_t3;
double left_v0,left_v1,left_v2,left_v3,right_v0,right_v1,right_v2,right_v3;
// double left_wheel_v,right_wheel_v;s

ros::Publisher right_cam_msg_pub,left_cam_msg_pub;
ros::Publisher right_wheel_pub, left_wheel_pub;
ros::Publisher left_hip_torque_pub,left_leg1_torque_pub,left_leg2_torque_pub,right_hip_torque_pub,right_leg1_torque_pub,right_leg2_torque_pub;

can_msgs::Frame generate_canFrame(uint16_t canid,double pos,double vel,double cur,uint8_t ack_model){
    can_msgs::Frame canFrame;
    canFrame.is_error = 0;
    canFrame.is_extended = 0;
    canFrame.is_rtr = 0;
    canFrame.id = canid;

    if(ack_model==1){
        canFrame.dlc = 7;
        canFrame.data.at(0) = 0x20;

        uint16_t pos_uint16 = uint16_t(65535*((pos+12.5)/25.0));
        canFrame.data.at(1) = pos_uint16 >> 8;
        canFrame.data.at(2) = pos_uint16 & 0x00FF;

        uint16_t vel_uint12 = uint16_t(4095*((vel+18.0)/36.0));
        uint16_t cur_uint12 = uint16_t(4095*((vel+30.0)/60.0));
        canFrame.data.at(3) = vel_uint12 >> 4;
        canFrame.data.at(4) = ((vel_uint12 << 4) | (cur_uint12>>8)) & 0x00FF;
        canFrame.data.at(5) = cur_uint12 & 0x00FF;
        canFrame.data.at(6) = 45*2+50; // temperature  : 45deg
    }
    else if(ack_model==3){
        canFrame.dlc = 8;
        canFrame.data.at(0) = 0x60;

        typeconverter.to_float = radps_2_rpm(vel);
        canFrame.data.at(1) = typeconverter.buf[3];
        canFrame.data.at(2) = typeconverter.buf[2];
        canFrame.data.at(3) = typeconverter.buf[1];
        canFrame.data.at(4) = typeconverter.buf[0]; 
        int16_t current_int = int16_t(cur) * 100;
        canFrame.data.at(5) = current_int >> 8;   
        canFrame.data.at(6) = current_int & 0x00FF;
        canFrame.data.at(7) = 45*2+50;
    }
    return canFrame;
}
void joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg)
{

    for(int i=0;i<msg->name.size();i++){
        if(msg->name.at(i)=="joint_base_left_hip"){
            left_q0 = msg->position.at(i);
            left_t0 = msg->effort.at(i);
            left_v0 = msg->velocity.at(i);
        }        
        else if(msg->name.at(i)=="joint_left_hip_left_leg1"){
            left_q1 = msg->position.at(i);
            left_t1 = msg->effort.at(i);
            left_v1 = msg->velocity.at(i);
        }
        else if(msg->name.at(i)=="joint_left_knee_leg2"){
            left_q2 = msg->position.at(i);
            left_t2 = msg->effort.at(i);
            left_v2 = msg->velocity.at(i);
        }
        else if(msg->name.at(i)=="joint_left_leg2_wheel"){
            left_q3 = msg->position.at(i);
            left_t3 = msg->effort.at(i);
            left_v3 = msg->velocity.at(i);
        }
        else if(msg->name.at(i)=="joint_base_right_hip"){
            right_q0 = msg->position.at(i);
            right_t0 = msg->effort.at(i);
            right_v0 = msg->velocity.at(i);
        }           
        else if(msg->name.at(i)=="joint_right_hip_right_leg1"){
            right_q1 = msg->position.at(i);
            right_t1 = msg->effort.at(i);
            right_v1 = msg->velocity.at(i);
        }   
        else if(msg->name.at(i)=="joint_right_knee_leg2"){
            right_q2 = msg->position.at(i);
            right_t2 = msg->effort.at(i);
            right_v2 = msg->velocity.at(i);
        }
        else if(msg->name.at(i)=="joint_right_leg2_wheel"){
            right_q3 = msg->position.at(i);
            right_t3 = msg->effort.at(i);
            right_v3 = msg->velocity.at(i);
        }
    }

}
void left_can_cmd_cb(const can_msgs::FrameConstPtr msg){
    if(msg->dlc==3 && msg->data.at(0)==0x65){
        // standard torque cmd
        double torque = 0.01*int16_t(msg->data.at(1) << 8 | msg->data.at(2));
        std_msgs::Float64 cmd;
        if(msg->id==1){
            cmd.data = torque;
            left_hip_torque_pub.publish(cmd);
            left_cam_msg_pub.publish(generate_canFrame(1,left_q0,left_v0,left_t0,0x01));
        }
        else if(msg->id==2){
            cmd.data = torque;
            left_leg1_torque_pub.publish(cmd);   
            left_cam_msg_pub.publish(generate_canFrame(2,left_q1,left_v1,left_t1,0x01));         
        }
        else if(msg->id==3){
            cmd.data = torque;
            left_leg2_torque_pub.publish(cmd);
            left_cam_msg_pub.publish(generate_canFrame(3,left_q2,left_v2,left_t2,0x01));            
        }
        else if(msg->id==4){
            cmd.data = torque;
            left_wheel_pub.publish(cmd);
            left_cam_msg_pub.publish(generate_canFrame(4,left_q3,left_v3,left_t3,0x01));            
        }         
    }
    else if(msg->dlc==3 && msg->data.at(0)==0x67){
        // standard torque cmd
        double torque = 0.01*int16_t(msg->data.at(1) << 8 | msg->data.at(2));
        std_msgs::Float64 cmd;
        if(msg->id==1){
            cmd.data = torque;
            left_hip_torque_pub.publish(cmd);
            left_cam_msg_pub.publish(generate_canFrame(1,left_q0,left_v0,left_t0,0x03));
        }
        else if(msg->id==2){
            cmd.data = torque;
            left_leg1_torque_pub.publish(cmd);   
            left_cam_msg_pub.publish(generate_canFrame(2,left_q1,left_v1,left_t1,0x03));         
        }
        else if(msg->id==3){
            cmd.data = torque;
            left_leg2_torque_pub.publish(cmd);
            left_cam_msg_pub.publish(generate_canFrame(3,left_q2,left_v2,left_t2,0x03));            
        }
        else if(msg->id==4){
            cmd.data = torque;
            left_wheel_pub.publish(cmd);
            left_cam_msg_pub.publish(generate_canFrame(4,left_q3,left_v3,left_t3,0x03));            
        }         
    }
}
void right_can_cmd_cb(const can_msgs::FrameConstPtr msg){
    if(msg->dlc==3 && msg->data.at(0)==0x65){
        // standard torque cmd return 0x01 ack
        double torque = 0.01*int16_t(msg->data.at(1) << 8 | msg->data.at(2));
        std_msgs::Float64 cmd;
        if(msg->id==1){
            cmd.data = torque;
            right_hip_torque_pub.publish(cmd);
            right_cam_msg_pub.publish(generate_canFrame(1,right_q0,right_v0,right_t0,0x01));
        }
        else if(msg->id==2){
            cmd.data = torque;
            right_leg1_torque_pub.publish(cmd);
            right_cam_msg_pub.publish(generate_canFrame(2,right_q1,right_v1,right_t1,0x01));            
        }
        else if(msg->id==3){
            cmd.data = torque;
            right_leg2_torque_pub.publish(cmd);   
            right_cam_msg_pub.publish(generate_canFrame(3,right_q2,right_v2,right_t2,0x01));           
        }
        else if(msg->id==4){
            cmd.data = torque;
            right_wheel_pub.publish(cmd);   
            right_cam_msg_pub.publish(generate_canFrame(4,right_q3,right_v3,right_t3,0x01));        
        }                
    }
    else if(msg->dlc==3 && msg->data.at(0)==0x67){
        // standard torque cmd return 0x03 ack
        double torque = 0.01*int16_t(msg->data.at(1) << 8 | msg->data.at(2));
        std_msgs::Float64 cmd;
        if(msg->id==1){
            cmd.data = torque;
            right_hip_torque_pub.publish(cmd);
            right_cam_msg_pub.publish(generate_canFrame(1,right_q0,right_v0,right_t0,0x03));
        }
        else if(msg->id==2){
            cmd.data = torque;
            right_leg1_torque_pub.publish(cmd);
            right_cam_msg_pub.publish(generate_canFrame(2,right_q1,right_v1,right_t1,0x03));            
        }
        else if(msg->id==3){
            cmd.data = torque;
            right_leg2_torque_pub.publish(cmd);   
            right_cam_msg_pub.publish(generate_canFrame(3,right_q2,right_v2,right_t2,0x03));           
        }
        else if(msg->id==4){
            cmd.data = torque;
            right_wheel_pub.publish(cmd);   
            right_cam_msg_pub.publish(generate_canFrame(4,right_q3,right_v3,right_t3,0x03));        
        }                
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_can_bridge_node");
    ros::NodeHandle n;
    ros::Subscriber joint_state_sub = n.subscribe("/od_robot/joint_states", 10, joint_state_cb);
    ros::Subscriber left_can_cmd_sub = n.subscribe("/left_can_cmd", 10, left_can_cmd_cb);
    ros::Subscriber right_can_cmd_sub = n.subscribe("/right_can_cmd", 10, right_can_cmd_cb);

    left_cam_msg_pub = n.advertise<can_msgs::Frame>("/left_can_msg", 10);
    right_cam_msg_pub = n.advertise<can_msgs::Frame>("/right_can_msg", 10);

    left_hip_torque_pub =  n.advertise<std_msgs::Float64>("/od_robot/base_left_hip_controller/command",10);
    left_leg1_torque_pub  = n.advertise<std_msgs::Float64>("/od_robot/left_hip_left_leg1_controller/command", 10);
    left_leg2_torque_pub  = n.advertise<std_msgs::Float64>("/od_robot/left_knee_leg2_controller/command", 10);
    right_hip_torque_pub =  n.advertise<std_msgs::Float64>("/od_robot/base_right_hip_controller/command",10);
    right_leg1_torque_pub  = n.advertise<std_msgs::Float64>("/od_robot/right_hip_right_leg1_controller/command", 10);
    right_leg2_torque_pub  = n.advertise<std_msgs::Float64>("/od_robot/right_knee_leg2_controller/command", 10);

    right_wheel_pub = n.advertise<std_msgs::Float64>("/od_robot/right_wheel_effort_controller/command", 10);
    left_wheel_pub = n.advertise<std_msgs::Float64>("/od_robot/left_wheel_effort_controller/command", 10);

    ros::spin();
}