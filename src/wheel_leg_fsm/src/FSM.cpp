#include "FSM.h"
#include <ros/ros.h>
#include "sbus_serial/Sbus.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "kinematics_3dof.h"
#include "can_msgs/Frame.h"
#include "sensor_msgs/JointState.h"
#include "Timer.hpp"

inline double rad_2_deg(double rad){
    return (rad / 3.1415926) * 180;
}
inline double deg_2_rad(double deg){
    return (deg / 180) * 3.1415926;
}
double clamp(double data,double upper,double lower);

FSM::FSM(WLParam &param_,Controller &controller_,Observer &observer_,Ratio &ratio_,Regulator &regulator_) : param(param_) ,controller(controller_),observer(observer_),ratio(ratio_),regulator(regulator_)
{
    _state = OB;
    _last_state = OB;
    if(param.isSim){
        ROS_INFO_STREAM("---------Simulation---------");
        left_motor_1.init( "left" ,1,0,0,1,0x01,0.4,-0.4,7);
        left_motor_2.init( "left" ,2,0,0,1,0x01,1.6,-0.5,2);
        left_motor_3.init( "left" ,3,0,0,1,0x01,-1.6,-2.7,15);
        left_motor_4.init( "left" ,4,0,0,1,0x03,-1,0,8);
        right_motor_1.init("right",1,0,0,1,0x01,0.4,-0.4,7);
        right_motor_2.init("right",2,0,0,1,0x01,1.6,-0.5,2);
        right_motor_3.init("right",3,0,0,1,0x01,-1.6,-2.7,15);
        right_motor_4.init("right",4,0,0,1,0x03,-1,0,8);
    }
    else{
        ROS_INFO_STREAM("---------Real World---------");
        left_motor_1.init( "left" ,1 ,0 , 0             ,0 , 0x01 , 0.4 , -0.4, 8);
        left_motor_2.init( "left" ,2 ,0 , 3.1415/2      ,1 , 0x01 , 1.6 , -0.5, 4);
        left_motor_3.init( "left" ,3 ,0 , -0.9477-0.5801,1 , 0x01 , -1.6, -2.7, 20);
        left_motor_4.init( "left" ,4 ,0 , 0             ,0 , 0x03 , -1  , 0   , 12);
        right_motor_1.init("right",1 ,0 , 0             ,0 , 0x01 , 0.4 , -0.4, 8);
        right_motor_2.init("right",2 ,0 , 3.1415/2      ,0 , 0x01 , 1.6 , -0.5, 4);
        right_motor_3.init("right",3 ,0 , -0.9477-0.5801,0 , 0x01 , -1.6, -2.7, 20);
        right_motor_4.init("right",4 ,0 , 0             ,0 , 0x03 , -1  , 0   , 12);
    }
    Motors.push_back(&left_motor_1);
    Motors.push_back(&left_motor_2);
    Motors.push_back(&left_motor_3);
    Motors.push_back(&left_motor_4);
    Motors.push_back(&right_motor_1);
    Motors.push_back(&right_motor_2);
    Motors.push_back(&right_motor_3);
    Motors.push_back(&right_motor_4);
    debug_sub_msg.data.push_back(0);
}

FSM::~FSM()
{
}


void FSM::_change_state(const ros::Time &now_time){
    switch (_state){
        case OB:{
            if(ratio.C > -1){
                if(!motor_is_received(now_time)){
                    ROS_WARN_THROTTLE(2,"motor msg unreceived ,cannot change to STAND/BALANCE mode.");
                }
                else{
                    _state = STAND;
                    ROS_INFO("\033[32m OB -->> STAND \033[32m");
                }
            }
            break;
        }
        case STAND:{
            if(observer.fall_flag == true){
                _state = FALL;
                ROS_ERROR(" STAND -->> FALL");
            }
            else if(!motor_is_received(now_time)){
                _state = OB;
                ROS_ERROR(" STAND -->> OB");
            }
            else if(ratio.C == -1){
                _state = OB;
                ROS_INFO(" STAND -->> OB");
            }
            else if(ratio.C == 1){
                if(!imu_is_received(now_time)){
                    ROS_WARN_THROTTLE(1,"imu msg unreceived ,cannot change to BALANCE mode.");
                }
                else{
                    _state = BALANCE;
                    ROS_INFO("\033[32m STAND -->> BALANCE \033[32m");
                }
            }
            break;
        }
        case BALANCE:{
            if(observer.fall_flag == true){
                _state = FALL;
                controller.reset_intergrator();
                ROS_ERROR(" BALANCE -->> FALL");
            }
            else if(!imu_is_received(now_time)||!motor_is_received(now_time)){
                _state = STAND;
                controller.reset_intergrator();
                ROS_ERROR(" BALANCE -->> STAND");
            }
            else if(ratio.C < 1){
                _state = STAND;
                controller.reset_intergrator();
                ROS_INFO(" BALANCE -->> STAND");
            }
            else if(ratio.B > -1){
                if(!pos_is_received(now_time)){
                    ROS_WARN_THROTTLE(1,"position msg unreceived ,cannot change to POSITION mode.");
                }
                else{
                    _state = POSITION;
                    controller.reset_intergrator();
                    ROS_INFO("\033[36m BALANCE -->> POSITION \033[36m");
                }
            }
            break;
        }
        case POSITION:{
            if(observer.fall_flag == true){
                _state = FALL;
                ROS_ERROR(" POSITION -->> FALL");
            }
            else if(!imu_is_received(now_time)||!motor_is_received(now_time)){
                _state = STAND;
                ROS_ERROR(" POSITION -->> STAND");
            }
            else if(ratio.C < 1){
                _state = STAND;
                ROS_INFO(" POSITION -->> STAND");
            }
            else if(!pos_is_received(now_time)){
                _state = BALANCE;
                ROS_WARN(" POSITION -->> BALANCE");
            }
            else if(ratio.B == -1){
                _state = BALANCE;
                ROS_INFO(" POSITION -->> BALANCE");
            }
            break;
        }
        case FALL:{
            if(ratio.C == -1 && observer.fall_flag == false){
                _state = OB;
                ROS_INFO(" FALL -->> OB");
            }
            break;
        }
        default:
            break;
    }
}

void FSM::loop()
{   
    // static long loop_cnt = 0 ;
    // Timer run_time;
    ros::Time now_time = ros::Time::now();
    // observer.step();
    // ROS_INFO_STREAM_THROTTLE(0.5,"observe cost "<<run_time.getMs()<<" ms");
    // run_time.start();

    std_msgs::Float64MultiArray debug_pub_msg;

    // ROS_INFO_STREAM_THROTTLE(0.5,"A: "<<ratio.A<<" B: "<<ratio.B<<" C: "<<ratio.C<<" D: "<<ratio.D);
    // ROS_INFO_STREAM_THROTTLE(0.5,"RV: "<<ratio.right_v<<" RH: "<<ratio.right_h<<" RK: "<< ratio.right_knob);
    // ROS_INFO_STREAM_THROTTLE(0.5,"LV: "<<ratio.left_v<<" LH: "<<ratio.left_h<<" LK: "<< ratio.left_knob);
    // do control calculate
    switch (_state){
        case OB:{
            leg_force_mutex.lock();
            leg_force.setZero();
            leg_force_mutex.unlock();
            controller.reset_intergrator();
            break;
        }
        case STAND:{
            if(!rc_is_received(now_time)){
                ROS_ERROR_THROTTLE(2,"no RC received. Be careful !");
            }
            
            // Eigen::Matrix<double,8,1> leg_angle = controller.pid_stand_calc(observer,ratio,debug_pub_msg);


            // double Kp = controller.output_ratio * 40;
            // double Kd = controller.output_ratio * 1.5;
            // Eigen::Vector3d right_feed_torque = controller.output_ratio *observer.right_Jp.transpose()*Eigen::Vector3d(0,0,-0);
            // Eigen::Vector3d left_feed_torque = controller.output_ratio *observer.left_Jp.transpose()*Eigen::Vector3d(0,0,-0);
            // if(ratio.A == 1){
            //     left_can_pub.publish(left_motor_1.set_mix(leg_angle(0),0,left_feed_torque(0),20,2));
            //     left_can_pub.publish(left_motor_2.set_mix(leg_angle(1),0,left_feed_torque(1),15,1.5));
            //     left_can_pub.publish(left_motor_3.set_mix(leg_angle(2),0,left_feed_torque(2),10,0.5));
            //     // left_can_pub.publish(left_motor_4.set_tor(controller.output_ratio * leg_angle(3)));
            //     left_can_pub.publish(left_motor_4.set_tor(0));

            //     right_can_pub.publish(right_motor_1.set_mix(leg_angle(4),0,right_feed_torque(0),20,2));
            //     right_can_pub.publish(right_motor_2.set_mix(leg_angle(5),0,right_feed_torque(1),15,1.5));
            //     right_can_pub.publish(right_motor_3.set_mix(leg_angle(6),0,right_feed_torque(2),10,0.5));
            //     // right_can_pub.publish(right_motor_4.set_tor(controller.output_ratio *leg_angle(7)));
            //     right_can_pub.publish(right_motor_4.set_tor(0));
            // }
            // else if(ratio.A == -1){
            //     left_can_pub.publish(left_motor_1.set_tor(0.0));
            //     left_can_pub.publish(left_motor_2.set_tor(0.0));
            //     left_can_pub.publish(left_motor_3.set_tor(0.0));
            //     left_can_pub.publish(left_motor_4.set_tor(0.0));

            //     right_can_pub.publish(right_motor_1.set_tor(0.0));
            //     right_can_pub.publish(right_motor_2.set_tor(0.0));
            //     right_can_pub.publish(right_motor_3.set_tor(0.0));
            //     right_can_pub.publish(right_motor_4.set_tor(0.0));
            //     controller.reset_intergrator();
            // }
            break;
        }
        case BALANCE:{
            // stay where it is
            if(!rc_is_received(now_time)){
                ROS_ERROR_THROTTLE(2,"no RC received. STAY !");
            }
            // controller._ocp.setStateMatrix(observer.rl(2),observer.rr(2),1/param.ctrl_freq_max);
            // Eigen::MatrixXd K;
            // if(controller._ocp.dlqr_calc(&K,1e-7)){
            //     controller.K_mutex.lock();
            //     controller.K_DOUBLE = K.block<8,14>(0,0);
            //     controller.Kc_DOUBLE = -K.block<8,3>(0,14);
            //     controller.K_mutex.unlock();
            // }
            break;
        }
        case POSITION:{
            // stay where it is
            if(!rc_is_received(now_time)){
                ROS_ERROR_THROTTLE(2,"no RC received. STAY !");
            }
            Timer run_time;
            Eigen::Matrix<double,8,1> tmp_leg_force = clamp(0.5*(ratio.left_knob + 1.0),1.0,0.0)*controller.mpc_calc(observer,ratio,regulator,debug_pub_msg);
            // Eigen::Matrix<double,8,1> tmp_leg_force = clamp(0.5*(ratio.left_knob + 1.0),1.0,0.0)*controller.lqr_calc(observer,ratio,regulator);
            // ROS_INFO_STREAM_THROTTLE(0.5,"mpc calc time: "<<run_time.getMs());
            leg_force_mutex.lock();
            leg_force = tmp_leg_force;
            leg_force_mutex.unlock();

            break;
        }
        case FALL:{
            leg_force_mutex.lock();
            leg_force.setZero();
            leg_force_mutex.unlock();
            break;
        }
        default:
            break;
    }
    
    /* --------- kalman test ----------*/
    // debug_pub_msg.data.push_back(observer.v_d(0));
    // debug_pub_msg.data.push_back(observer.v_d(1));
    // debug_pub_msg.data.push_back(observer.v_d(2));
    // debug_pub_msg.data.push_back(observer.v_I(0));
    // debug_pub_msg.data.push_back(observer.v_I(1));
    // debug_pub_msg.data.push_back(observer.v_I(2));
    // debug_pub_msg.data.push_back(observer.v_d_ground_truth(0));
    // debug_pub_msg.data.push_back(observer.v_d_ground_truth(1));
    // debug_pub_msg.data.push_back(observer.v_d_ground_truth(2));
    // debug_pub_msg.data.push_back(observer.v_d_estimated_by_leg(0));
    // debug_pub_msg.data.push_back(observer.v_d_estimated_by_leg(1));
    // debug_pub_msg.data.push_back(observer.v_d_estimated_by_leg(2));
    // debug_pub_msg.data.push_back(observer.v_d(0));
    // debug_pub_msg.data.push_back(observer.v_d(1));
    // debug_pub_msg.data.push_back(observer.v_d(2));
    
    /* --------- integration test ----------*/
    // debug_pub_msg.data.push_back(controller.vx_inte_err);
    // debug_pub_msg.data.push_back(controller.height_inte_err);
    // debug_pub_msg.data.push_back(controller.yaw_rate_inte_err);    

    _change_state(now_time);
    // debug_pub.publish(debug_pub_msg);
    publish_joint_state(10);
    publish_wrench(20);
    _last_state = _state;

    // if(++loop_cnt  % 2000 == 1){
    //     regulator.start(Regulator::Traj_t::CIRCLE, observer, 10);
    //     ROS_INFO("start_regulator");
    // }
    // ROS_INFO_STREAM_THROTTLE(0.5,"calc and pub cost "<<run_time.getMs()<<" ms");
}

bool FSM::rc_is_received(const ros::Time &now_time){
    return (now_time - _rc_rcv_stamp).toSec() < param.msg_timeout.rc;
}
bool FSM::imu_is_received(const ros::Time &now_time){
    return (now_time - _imu_rcv_stamp).toSec() < param.msg_timeout.imu;
}
bool FSM::motor_is_received(const ros::Time &now_time){
    // return true;
    if((now_time - std::max(_left_motor_rcv_stamp,_right_motor_rcv_stamp)).toSec() > param.msg_timeout.motor){
        ROS_ERROR_STREAM_ONCE("some motor unreceived !");
        return false;
    }
    return true;
    // return (now_time - std::min(_left_motor_rcv_stamp,_right_motor_rcv_stamp)).toSec() < param.msg_timeout.motor;
}
bool FSM::pos_is_received(const ros::Time &now_time){
    return (now_time - _pos_rcv_stamp).toSec() < param.msg_timeout.pos;
}
//调试区
void FSM::cmd_cvel(geometry_msgs::Twist vel_msg){
   // _vel_rcv_stamp = ros::Time::now();
    ratio.dx = vel_msg.linear.x;
    ratio.dw = vel_msg.angular.z;
}
//
void FSM::imu_cb(sensor_msgs::ImuConstPtr msg){
    _imu_rcv_stamp = ros::Time::now();
    observer.update(msg);
}
void FSM::rc_cb(sbus_serial::SbusConstPtr msg){
    _rc_rcv_stamp = ros::Time::now();
    ratio.update(msg);
}
void FSM::ground_truth_cb(const nav_msgs::Odometry::ConstPtr msg){
    _pos_rcv_stamp = ros::Time::now();
    observer.update(msg);
}
void FSM::vio_cb(const nav_msgs::Odometry::ConstPtr msg){
    _pos_rcv_stamp = ros::Time::now();
    observer.vio_update(msg);
}
void FSM::vicon_cb(const geometry_msgs::PoseStampedConstPtr msg){
    _pos_rcv_stamp = ros::Time::now();
    observer.vicon_update(msg);
}
void FSM::left_can_cb(can_msgs::Frame::ConstPtr msg){
    _left_motor_rcv_stamp = ros::Time::now();
    if(msg->id == 0x001 || msg->id == 0x205){
        left_motor_1.update(msg);
        left_motor1_pub.publish(left_motor_1.get_ros_msg());
        observer.update(left_motor_1);
    }
    else if(msg->id == 0x002 || msg->id == 0x206){
        left_motor_2.update(msg);
        left_motor2_pub.publish(left_motor_2.get_ros_msg());
        observer.update(left_motor_2);
    }
    else if(msg->id == 0x003 || msg->id == 0x207){
        left_motor_3.update(msg);
        left_motor3_pub.publish(left_motor_3.get_ros_msg());
        observer.update(left_motor_3);
    }
    else if(msg->id == 0x004 || msg->id == 0x208){
        left_motor_4.update(msg);
        left_motor4_pub.publish(left_motor_4.get_ros_msg());
        observer.update(left_motor_4);
    }
}

void FSM::right_can_cb(can_msgs::Frame::ConstPtr msg){
    _right_motor_rcv_stamp = ros::Time::now();
    if(msg->id == 0x001 || msg->id == 0x205){
        right_motor_1.update(msg);
        right_motor1_pub.publish(right_motor_1.get_ros_msg());
        observer.update(right_motor_1);
    }
    else if(msg->id == 0x002 || msg->id == 0x206){
        right_motor_2.update(msg);
        right_motor2_pub.publish(right_motor_2.get_ros_msg());
        observer.update(right_motor_2);
    }
    else if(msg->id == 0x003 || msg->id == 0x207){
        right_motor_3.update(msg);
        right_motor3_pub.publish(right_motor_3.get_ros_msg());
        observer.update(right_motor_3);
    }
    else if(msg->id == 0x004 || msg->id == 0x208){
        right_motor_4.update(msg);
        right_motor4_pub.publish(right_motor_4.get_ros_msg());   
        observer.update(right_motor_4);
    }
}

void FSM::debug_cb(std_msgs::Float64MultiArray::ConstPtr msg){
    debug_sub_msg = *msg;
}
void FSM::publish_joint_state(double hz){
    static ros::Time pub_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    if((now_time-pub_time).toSec() > (1.0/hz)){
        sensor_msgs::JointState msg;
        msg.header.stamp = now_time;
        const static std::vector<std::string> joint_names = {"joint_base_left_hip","joint_left_hip_left_leg1","joint_left_knee_leg2","joint_left_leg2_wheel",
        "joint_base_right_hip","joint_right_hip_right_leg1","joint_right_knee_leg2","joint_right_leg2_wheel"};
        for(int i=0;i<8;i++){
            msg.name.push_back(joint_names[i]);
            msg.position.push_back(Motors[i]->position);
            msg.velocity.push_back(Motors[i]->velocity);
            msg.effort.push_back(Motors[i]->current);
        }
        joint_state_pub.publish(msg);
        pub_time = now_time;
    }
}

void FSM::publish_wrench(double hz){
    static ros::Time pub_time = ros::Time::now();
    static tf2_ros::TransformBroadcaster tf_pub;
    ros::Time now_time = ros::Time::now();
    if((now_time-pub_time).toSec() > (1.0/hz)){
        // first pub the Frame
        Eigen::Quaterniond q_rp(observer.rotation_rp);
        geometry_msgs::TransformStamped left_trans,right_trans;
        left_trans.header.stamp = now_time;
        left_trans.header.frame_id = "base_footprint";
        left_trans.child_frame_id = "left_contact";
        left_trans.transform.translation.x = observer.rlb(0);
        left_trans.transform.translation.y = observer.rlb(1);
        left_trans.transform.translation.z = observer.rlb(2);
        left_trans.transform.rotation.w = q_rp.w();
        left_trans.transform.rotation.x = -q_rp.x();
        left_trans.transform.rotation.y = -q_rp.y();
        left_trans.transform.rotation.z = -q_rp.z();

        right_trans.header.stamp = now_time;
        right_trans.header.frame_id = "base_footprint";
        right_trans.child_frame_id = "right_contact";
        right_trans.transform.translation.x = observer.rrb(0);
        right_trans.transform.translation.y = observer.rrb(1);
        right_trans.transform.translation.z = observer.rrb(2);
        
        right_trans.transform.rotation.w = q_rp.w();
        right_trans.transform.rotation.x = -q_rp.x();
        right_trans.transform.rotation.y = -q_rp.y();
        right_trans.transform.rotation.z = -q_rp.z();
        tf_pub.sendTransform(left_trans);
        tf_pub.sendTransform(right_trans);
        // then pub the wrench msg
        left_wrench_pub.publish(controller.left_wrench);
        right_wrench_pub.publish(controller.right_wrench);

        pub_time = now_time;
    }
}

void FSM::publish_state(double hz){
    static ros::Time pub_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    std_msgs::Float64MultiArray state_msg;
    if((now_time-pub_time).toSec() > (1.0/hz)){
            Eigen::Matrix<double,17,1> show_state =  observer.get_mpc_state();
            state_msg.data.push_back(show_state(0));
            state_msg.data.push_back(show_state(1));
            state_msg.data.push_back(show_state(2));
            state_msg.data.push_back(show_state(3));
            state_msg.data.push_back(show_state(4));
            state_msg.data.push_back(show_state(5));
            state_msg.data.push_back(show_state(6));
            state_msg.data.push_back(show_state(7));
            state_msg.data.push_back(show_state(8));
            state_msg.data.push_back(show_state(9));
            state_msg.data.push_back(show_state(10));
            state_msg.data.push_back(show_state(11));
            state_msg.data.push_back(show_state(12));
            state_msg.data.push_back(show_state(13));
            state_msg.data.push_back(show_state(14));
            state_msg.data.push_back(show_state(15));
            state_msg.data.push_back(show_state(16));
            state_pub.publish(state_msg);
            pub_time = now_time;
    }
}

void FSM::wbc_calc(){
    std_msgs::Float64MultiArray debug_pub_msg;
    if(_state == POSITION){
        leg_force_mutex.lock();
        Eigen::Vector3d left_leg_torque = observer.left_Jp.transpose()*observer.rotation_rp.transpose()*Eigen::Vector3d(-leg_force(0),-leg_force(1),-leg_force(2));
        Eigen::Vector3d right_leg_torque = observer.right_Jp.transpose()*observer.rotation_rp.transpose()*Eigen::Vector3d(-leg_force(4),-leg_force(5),-leg_force(6));
        left_leg_torque += observer.left_Jr.transpose()*observer.rotation_rp.transpose()*Eigen::Vector3d(0,-leg_force(3),0);
        right_leg_torque += observer.right_Jr.transpose()*observer.rotation_rp.transpose()*Eigen::Vector3d(0,-leg_force(7),0);  
        leg_force_mutex.unlock();  
        // left_leg_torque,-leg_force(3),right_leg_torque,-leg_force(7);
        if(ratio.A == 1){
            left_can_pub.publish(left_motor_1.set_tor(left_leg_torque(0)));
            left_can_pub.publish(left_motor_2.set_tor(left_leg_torque(1)));
            left_can_pub.publish(left_motor_3.set_tor(left_leg_torque(2)));
            left_can_pub.publish(left_motor_4.set_tor(-leg_force(3)));

            right_can_pub.publish(right_motor_1.set_tor(right_leg_torque(0)));
            right_can_pub.publish(right_motor_2.set_tor(right_leg_torque(1)));
            right_can_pub.publish(right_motor_3.set_tor(right_leg_torque(2)));
            right_can_pub.publish(right_motor_4.set_tor(-leg_force(7)));
        }
        else if(ratio.A == -1){
            left_can_pub.publish(left_motor_1.set_tor(0.0));
            left_can_pub.publish(left_motor_2.set_tor(0.0));
            left_can_pub.publish(left_motor_3.set_tor(0.0));
            left_can_pub.publish(left_motor_4.set_tor(0.0));

            right_can_pub.publish(right_motor_1.set_tor(0.0));
            right_can_pub.publish(right_motor_2.set_tor(0.0));
            right_can_pub.publish(right_motor_3.set_tor(0.0));
            right_can_pub.publish(right_motor_4.set_tor(0.0));
            controller.reset_intergrator();
        }
    }
    else if(_state == BALANCE){
        Eigen::Matrix<double,8,1> tor =  clamp(0.5*(ratio.left_knob + 1.0),1.0,0.0)*controller.lqr_calc(observer,ratio,regulator,debug_pub_msg);
        if(ratio.A == 1){
            left_can_pub.publish(left_motor_1.set_tor(tor(0)));
            left_can_pub.publish(left_motor_2.set_tor(tor(1)));
            left_can_pub.publish(left_motor_3.set_tor(tor(2)));
            left_can_pub.publish(left_motor_4.set_tor(tor(3)));

            right_can_pub.publish(right_motor_1.set_tor(tor(4)));
            right_can_pub.publish(right_motor_2.set_tor(tor(5)));
            right_can_pub.publish(right_motor_3.set_tor(tor(6)));
            right_can_pub.publish(right_motor_4.set_tor(tor(7)));
        }
        else if(ratio.A == -1){
            left_can_pub.publish(left_motor_1.set_tor(0.0));
            left_can_pub.publish(left_motor_2.set_tor(0.0));
            left_can_pub.publish(left_motor_3.set_tor(0.0));
            left_can_pub.publish(left_motor_4.set_tor(0.0));

            right_can_pub.publish(right_motor_1.set_tor(0.0));
            right_can_pub.publish(right_motor_2.set_tor(0.0));
            right_can_pub.publish(right_motor_3.set_tor(0.0));
            right_can_pub.publish(right_motor_4.set_tor(0.0));
            controller.reset_intergrator();
        }
        debug_pub.publish(debug_pub_msg);
    }
    else if(_state==STAND){
        Eigen::Matrix<double,8,1> tor =  clamp(0.5*(ratio.left_knob + 1.0),1.0,0.0)*controller.balance_stand_calc(observer,ratio,regulator);
        if(ratio.A == 1){
            left_can_pub.publish(left_motor_1.set_tor(tor(0)));
            left_can_pub.publish(left_motor_2.set_tor(tor(1)));
            left_can_pub.publish(left_motor_3.set_tor(tor(2)));
            left_can_pub.publish(left_motor_4.set_tor(tor(3)));

            right_can_pub.publish(right_motor_1.set_tor(tor(4)));
            right_can_pub.publish(right_motor_2.set_tor(tor(5)));
            right_can_pub.publish(right_motor_3.set_tor(tor(6)));
            right_can_pub.publish(right_motor_4.set_tor(tor(7)));
        }
        else if(ratio.A == -1){
            left_can_pub.publish(left_motor_1.set_tor(0.0));
            left_can_pub.publish(left_motor_2.set_tor(0.0));
            left_can_pub.publish(left_motor_3.set_tor(0.0));
            left_can_pub.publish(left_motor_4.set_tor(0.0));

            right_can_pub.publish(right_motor_1.set_tor(0.0));
            right_can_pub.publish(right_motor_2.set_tor(0.0));
            right_can_pub.publish(right_motor_3.set_tor(0.0));
            right_can_pub.publish(right_motor_4.set_tor(0.0));
            controller.reset_intergrator();
        }    
    }
    else if(_state == OB || _state==FALL){
        left_can_pub.publish(left_motor_1.set_tor(0.0));
        left_can_pub.publish(left_motor_2.set_tor(0.0));
        left_can_pub.publish(left_motor_3.set_tor(0.0));
        left_can_pub.publish(left_motor_4.set_tor(0.0));

        right_can_pub.publish(right_motor_1.set_tor(0.0));
        right_can_pub.publish(right_motor_2.set_tor(0.0));
        right_can_pub.publish(right_motor_3.set_tor(0.0));
        right_can_pub.publish(right_motor_4.set_tor(0.0));
        controller.reset_intergrator();
    }

}