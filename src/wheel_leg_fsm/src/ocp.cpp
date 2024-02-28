#include "ocp.h"
#include "Timer.hpp"
const double INF = 1e6;
double clamp(double data,double upper,double lower);
double yaw_error(double yaw1,double yaw2);
ocp::ocp(WLParam param_,int state_dim,int input_dim,int constrain_dim) :param(param_)
{   
    x_dim = state_dim;
    u_dim = input_dim;
    c_dim = constrain_dim;
    setStateMatrix(-0.4,-0.4,1.0 / param.ctrl_freq_max);
    malloced = false;
}

ocp::~ocp()
{

}

void ocp::setStateMatrix(double rlz_,double rrz_,double Ts){
    double R=0.06,D=0.372,m=1.05,M=8.59,g=9.83;
    double rlz=clamp(rlz_,-0.2,-0.45);
    double rrz=clamp(rrz_,-0.2,-0.45);
    double wz=0.0;

    double Pl=M*g/2,Pr=M*g/2,Nl=0.0,Nr=0.0,Ql=0.101 / rlz * M*g/2,Qr=-0.101 / rrz * M*g/2;
    double Ixx,Iyy,Izz,Iwy,Iwz;

    if(param.isSim){
        Ixx = 0.010105283414695678+0.0086*2;
        Iyy = 0.008445352479158024;
        Izz = 0.011121247778265564+0.0079*2;
        Iwy = 0.002498809975509641;
        Iwz = 0.0014518643717015625+0.0008042358709478445*2;
    }
    else{
        Ixx = 0.09586;
        Iyy = 0.03247;
        Izz = 0.09988+0.036*2;
        Iwy = 0.00139;
        Iwz = 0.000834;
    }
    A_c.resize(14,14);
    A_c.setZero();
    A_c(1,2)=1;
    A_c(3,4)=1;
    A_c(5,6)=1;
    A_c(6,1)=-(Pl + Pr - 2*R*m*wz*wz)/Ixx;
    A_c(6,10)=-(D*R*m*(Nl - Nr))/(2*Ixx*(2*Iwz + Izz));
    A_c(6,11)=(2*R*m*wz)/Ixx;
    A_c(6,12)=-(D*R*m*(Nl - Nr))/(2*Ixx*(2*Iwz + Izz));
    A_c(6,13)=(2*R*m*wz)/Ixx;
    A_c(7,8)=1;
    A_c(8,10)=-Pl/Iyy;
    A_c(8,12)=-Pr/Iyy;
    A_c(9,1)=(Nl + Nr)/(2*Iwz + Izz);
    A_c(9,10)=Ql/(2*Iwz + Izz);
    A_c(9,12)=Qr/(2*Iwz + Izz);
    A_c(10,11)=1;
    A_c(11,1)=(D*Nl)/(2*Iwz + Izz);
    A_c(11,2)=-(wz*(Iwy + 2*R*R*m))/(Iwy + R*R*m);
    A_c(11,10)=(m*(wz*wz + (D*Ql)/(2*(2*Iwz + Izz))) + (D*Iwy*Ql)/(2*R*R*(2*Iwz + Izz)))/(m + Iwy/R/R);
    A_c(11,12)=(D*Qr)/(4*Iwz + 2*Izz);
    A_c(12,13)=1;
    A_c(13,1)=-(D*Nr)/(2*Iwz + Izz);
    A_c(13,2)=-(wz*(Iwy + 2*R*R*m))/(Iwy + R*R*m);
    A_c(13,10)=-(D*Ql)/(4*Iwz + 2*Izz);
    A_c(13,12)=(m*(wz*wz - (D*Qr)/(2*(2*Iwz + Izz))) - (D*Iwy*Qr)/(2*R*R*(2*Iwz + Izz)))/(m + Iwy/R/R);

    B_c.resize(14,8);
    B_c.setZero();
    B_c(0,0)=1/M;
    B_c(0,4)=1/M;
    B_c(2,1)=1/M;
    B_c(2,5)=1/M;
    B_c(4,2)=1/M;
    B_c(4,6)=1/M;
    B_c(6,1)=(R - rlz)/Ixx;
    B_c(6,2)=D/(2*Ixx);
    B_c(6,5)=(R - rrz)/Ixx;
    B_c(6,6)=-D/(2*Ixx);
    B_c(8,0)=rlz/Iyy;
    B_c(8,3)=1/Iyy;
    B_c(8,4)=rrz/Iyy;
    B_c(8,7)=1/Iyy;
    B_c(9,0)=-D/(2*(2*Iwz + Izz));
    B_c(9,4)=D/(2*(2*Iwz + Izz));
    B_c(11,0)=-(m*(1/M + D*D/(4*(2*Iwz + Izz))) + (Iwy*(1/M + D*D/(4*(2*Iwz + Izz))))/R/R + 1)/(m + Iwy/R/R);
    B_c(11,3)=-R/(Iwy + R*R*m);
    B_c(11,4)=-(8*Iwz + 4*Izz - D*D*M)/(4*M*(2*Iwz + Izz));
    B_c(13,0)=-(8*Iwz + 4*Izz - D*D*M)/(4*M*(2*Iwz + Izz));
    B_c(13,4)=-(m*(1/M + D*D/(4*(2*Iwz + Izz))) + (Iwy*(1/M + D*D/(4*(2*Iwz + Izz))))/R/R + 1)/(m + Iwy/R/R);
    B_c(13,7)=-R/(Iwy + R*R*m);

    C_c.resize(3,14);
    C_c.setZero();
    C_c(0,0) = 1;
    C_c(1,9) = 1;
    C_c(2,3) = 1;

    D_c.resize(3,8);
    D_c.setZero();

    Eigen::Matrix3d A_i = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d B_i = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_i = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D_i = Eigen::Matrix3d::Zero();

    A_sc.resize(x_dim,x_dim);
    B_sc.resize(x_dim,u_dim);

    A_sc << A_c ,Eigen::MatrixXd::Zero(14,3), -B_i*C_c , A_i;
    B_sc << B_c,-B_i*D_c;
    // std::cout << "A_sc:"<< std::endl<< A_sc << std::endl;
    // std::cout << "B_sc:"<< std::endl<< B_sc << std::endl;
    // Discretization
    Eigen::MatrixXd tmp_AT,tmp_BT;
    tmp_AT.resize(x_dim,x_dim);
    tmp_BT.resize(x_dim,x_dim);
    tmp_AT.setIdentity();
    tmp_BT.setZero();

    A_sd.resize(x_dim,x_dim);
    A_sd.setZero();
    B_sd.resize(x_dim,u_dim);
    B_sd.setZero();

    for(int mi=1;mi<5;mi++){
        A_sd +=  tmp_AT;
        tmp_BT += tmp_AT * Ts / mi;
        if(tmp_AT.lpNorm<Eigen::Infinity>() < 1e-4){
            break;
        }
        tmp_AT = tmp_AT * A_sc * Ts / (mi);
    }
    // A_sd = Eigen::MatrixXd::Identity(x_dim,x_dim) + A_sc * Ts + 0.5*A_sc*A_sc*Ts*Ts;
    // B_sd = (Eigen::MatrixXd::Identity(x_dim,x_dim)* Ts + 0.5*A_sc*Ts*Ts)*B_sc;
    B_sd = tmp_BT * B_sc;
    // std::cout << "A_sd:"<< std::endl<< A_sd << std::endl;
    // std::cout << "B_sd:"<< std::endl<< B_sd << std::endl;
}
// all param 
void ocp::setStateMatrix(Observer obser,double Ts,Eigen::MatrixXd state_point,Eigen::MatrixXd input_point){
    double R=0.06,m=1.05,M=8.59,g=9.83;
    double D=obser.rl(1)-obser.rr(1);
    double rlz=clamp(obser.rl(2),-0.2,-0.45);
    double rrz=clamp(obser.rr(2),-0.2,-0.45);
    // double rlx=obser.rl(0);
    // double rrx=obser.rr(0);
    // double Drlx=obser.Drl(0);
    // double Drrx=obser.Drr(0);
    // double wbz=obser.angular_xyz(2);
    // double vbx = obser.v_d(0);
    // double yaw = obser.euler_rpy(2);
    // double y=-0.5*(obser.rl(1)+obser.rr(1));
    // double vy=-0.5*(obser.Drl(1)+obser.Drr(1));
    double rlx=0;
    double rrx=0;
    double Drlx=0;
    double Drrx=0;
    double wbz = 0;
    double vbx = 0;
    double yaw = obser.euler_rpy(2);
    double y=0;
    double vy=0;

    double Flz=M*g/2,Frz=M*g/2,Flx=0.0,Frx=0.0,Fly=0.5*M*vbx*wbz,Fry=0.5*M*vbx*wbz;//0.5*M*vbx*wbz
    double Tly=0,Try=0;
    double Ib_xx,Ib_yy,Ib_zz,Iw_yy,Iw_zz;
    double R_wheel = param.robot.wheel_radius;
    if(param.isSim){
        Ib_xx = 0.010105283414695678+0.0086*2;
        Ib_yy = 0.008445352479158024;
        Ib_zz = 0.011121247778265564+0.0079*2;
        Iw_yy = 0.002498809975509641;
        Iw_zz = 0.0014518643717015625+0.0008042358709478445*2;
    }
    else{
        Ib_xx = 0.09586;
        Ib_yy = 0.03247;
        Ib_zz = 0.09988+0.036*2;
        Iw_yy = 0.00139;
        Iw_zz = 0.000834;
    }
    A_c.resize(17,17);
    A_c.setZero();
    A_c(0,2)=cos(yaw) - Ts*wbz*sin(yaw);
    A_c(0,4)=-sin(yaw);
    A_c(0,11)=- vbx*sin(yaw) - cos(yaw)*(vy - (wbz*(rlx + rrx))/2 + Ts*vbx*wbz);
    A_c(0,12)=sin(yaw)*(rlx/2 + rrx/2 - Ts*vbx);
    A_c(0,13)=(wbz*sin(yaw))/2;
    A_c(0,15)=(wbz*sin(yaw))/2;
    A_c(1,2)=sin(yaw) + Ts*wbz*cos(yaw);
    A_c(1,4)=cos(yaw);
    A_c(1,11)=vbx*cos(yaw) - sin(yaw)*(vy - (wbz*(rlx + rrx))/2 + Ts*vbx*wbz);
    A_c(1,12)=-cos(yaw)*(rlx/2 + rrx/2 - Ts*vbx);
    A_c(1,13)=-(wbz*cos(yaw))/2;
    A_c(1,15)=-(wbz*cos(yaw))/2;
    A_c(3,4)=1;
    // A_c(4,3)=wbz*wbz + (rlx*(Flx + Frx))/(2*(Ib_zz + 2*Iw_zz)) + (rrx*(Flx + Frx))/(2*(Ib_zz + 2*Iw_zz));
    // A_c(4,12)=Drlx + Drrx + 2*wbz*y;
    // A_c(4,13)=(4*Fly*rlx + 2*Fly*rrx + 2*Fry*rrx + 2*Flx*y + 2*Frx*y - D*Flx + D*Frx)/(4*(Ib_zz + 2*Iw_zz));
    // A_c(4,14)=wbz;
    // A_c(4,15)=(2*Fly*rlx + 2*Fry*rlx + 4*Fry*rrx + 2*Flx*y + 2*Frx*y - D*Flx + D*Frx)/(4*(Ib_zz + 2*Iw_zz));
    // A_c(4,16)=wbz;
    A_c(4,2)=-wbz;
    A_c(4,3)=((rlx + rrx)*(Flx + Frx))/(2*(Ib_zz + 2*Iw_zz));
    A_c(4,12)=Drlx + Drrx - vbx;
    A_c(4,13)=(4*Fly*rlx + 2*Fly*rrx + 2*Fry*rrx + 2*Flx*y + 2*Frx*y - D*Flx + D*Frx)/(4*(Ib_zz + 2*Iw_zz));
    A_c(4,14)=wbz;
    A_c(4,15)=(2*Fly*rlx + 2*Fry*rlx + 4*Fry*rrx + 2*Flx*y + 2*Frx*y - D*Flx + D*Frx)/(4*(Ib_zz + 2*Iw_zz));
    A_c(4,16)=wbz;
    A_c(5,6)=1;
    A_c(7,8)=1;
    A_c(8,3)=-(Flz + Frz)/Ib_xx;
    A_c(9,10)=1;
    A_c(10,13)=-Flz/Ib_yy;
    A_c(10,15)=-Frz/Ib_yy;
    A_c(11,12)=1;
    A_c(12,3)=(Flx + Frx)/(Ib_zz + 2*Iw_zz);
    A_c(12,13)=Fly/(Ib_zz + 2*Iw_zz);
    A_c(12,15)=Fry/(Ib_zz + 2*Iw_zz);
    A_c(13,14)=1;
    A_c(14,3)=((D/2 - y)*(Flx + Frx))/(Ib_zz + 2*Iw_zz) - (Fly*rlx + Fry*rrx + Frx*(D/2 + y) - Flx*(D/2 - y))/(Ib_zz + 2*Iw_zz);
    A_c(14,4)=-2*wbz;
    A_c(14,12)=wbz*(rlx + rrx) - 2*vy - 2*wbz*(rlx/2 - rrx/2);
    A_c(14,13)=(Fly*(D/2 - y))/(Ib_zz + 2*Iw_zz);
    A_c(14,15)=wbz*wbz + (Fry*(D/2 - y))/(Ib_zz + 2*Iw_zz);
    A_c(15,16)=1;
    A_c(16,3)=- (Fly*rlx + Fry*rrx + Frx*(D/2 + y) - Flx*(D/2 - y))/(Ib_zz + 2*Iw_zz) - ((D/2 + y)*(Flx + Frx))/(Ib_zz + 2*Iw_zz);
    A_c(16,4)=-2*wbz;
    A_c(16,12)=wbz*(rlx + rrx) - 2*vy + 2*wbz*(rlx/2 - rrx/2);
    A_c(16,13)=wbz*wbz - (Fly*(D/2 + y))/(Ib_zz + 2*Iw_zz);
    A_c(16,15)=-(Fry*(D/2 + y))/(Ib_zz + 2*Iw_zz);

    B_c.resize(17,8);
    B_c.setZero();
    B_c(2,0)=1/M;
    B_c(2,4)=1/M;
    // B_c(4,0)=-((rlx + rrx)*(D - 2*y))/(4*(Ib_zz + 2*Iw_zz));
    // B_c(4,1)=(2*Ib_zz + 4*Iw_zz + M*rlx^2 + M*rlx*rrx)/(2*M*(Ib_zz + 2*Iw_zz));
    // B_c(4,4)=((rlx + rrx)*(D + 2*y))/(4*(Ib_zz + 2*Iw_zz));
    // B_c(4,5)=(2*Ib_zz + 4*Iw_zz + M*rrx^2 + M*rlx*rrx)/(2*M*(Ib_zz + 2*Iw_zz));
    B_c(4,0)=-((rlx + rrx)*(D - 2*y))/(4*(Ib_zz + 2*Iw_zz));
    B_c(4,1)=(2*Ib_zz + 4*Iw_zz + M*rlx*rlx + M*rlx*rrx)/(2*M*(Ib_zz + 2*Iw_zz));
    B_c(4,4)=((rlx + rrx)*(D + 2*y))/(4*(Ib_zz + 2*Iw_zz));
    B_c(4,5)=(2*Ib_zz + 4*Iw_zz + M*rrx*rrx + M*rlx*rrx)/(2*M*(Ib_zz + 2*Iw_zz));
    B_c(6,2)=1/M;
    B_c(6,6)=1/M;
    B_c(8,1)=(R_wheel - rlz)/Ib_xx;
    B_c(8,2)=(D/2 - y)/Ib_xx;
    B_c(8,5)=(R_wheel - rrz)/Ib_xx;
    B_c(8,6)=-(D/2 + y)/Ib_xx;
    B_c(10,0)=rlz/Ib_yy;
    B_c(10,2)=-rlx/Ib_yy;
    B_c(10,3)=1/Ib_yy;
    B_c(10,4)=rrz/Ib_yy;
    B_c(10,6)=-rrx/Ib_yy;
    B_c(10,7)=1/Ib_yy;
    B_c(12,0)=-(D/2 - y)/(Ib_zz + 2*Iw_zz);
    B_c(12,1)=rlx/(Ib_zz + 2*Iw_zz);
    B_c(12,4)=(D/2 + y)/(Ib_zz + 2*Iw_zz);
    B_c(12,5)=rrx/(Ib_zz + 2*Iw_zz);
    B_c(14,0)=- (D/2 - y)*(D/2 - y)/(Ib_zz + 2*Iw_zz) - 1/M - R_wheel/(R_wheel*m + Iw_yy/R_wheel);
    B_c(14,1)=(rlx*(D/2 - y))/(Ib_zz + 2*Iw_zz);
    B_c(14,3)=-1/(R_wheel*m + Iw_yy/R_wheel);
    B_c(14,4)=((D/2 + y)*(D/2 - y))/(Ib_zz + 2*Iw_zz) - 1/M;
    B_c(14,5)=(rrx*(D/2 - y))/(Ib_zz + 2*Iw_zz);
    B_c(16,0)=((D/2 + y)*(D/2 - y))/(Ib_zz + 2*Iw_zz) - 1/M;
    B_c(16,1)=-(rlx*(D/2 + y))/(Ib_zz + 2*Iw_zz);
    B_c(16,4)=- 1/M - R_wheel/(R_wheel*m + Iw_yy/R_wheel) - (D/2 + y)*(D/2 + y)/(Ib_zz + 2*Iw_zz);
    B_c(16,5)=-(rrx*(D/2 + y))/(Ib_zz + 2*Iw_zz);
    B_c(16,7)=-1/(R_wheel*m + Iw_yy/R_wheel);

    C_c.resize(3,17);
    C_c.setZero();
    C_c(0,2) = 1;
    C_c(1,3) = 1;
    C_c(2,12) = 1;
    // std::cout << "A_sc:"<< std::endl<< A_sc << std::endl;
    // std::cout << "B_sc:"<< std::endl<< B_sc << std::endl;
    // Discretization
    Eigen::MatrixXd tmp_AT,tmp_BT;
    tmp_AT.resize(x_dim,x_dim);
    tmp_BT.resize(x_dim,x_dim);
    tmp_AT.setIdentity();
    tmp_BT.setZero();

    A_sd.resize(x_dim,x_dim);
    A_sd.setZero();
    B_sd.resize(x_dim,u_dim);
    B_sd.setZero();

    // for(int mi=1;mi<5;mi++){
    //     A_sd +=  tmp_AT;
    //     tmp_BT += tmp_AT * Ts / mi;
    //     if(tmp_AT.lpNorm<Eigen::Infinity>() < 1e-4){
    //         break;
    //     }
    //     tmp_AT = tmp_AT * A_c * Ts / (mi);
    // }
    A_sd = Eigen::MatrixXd::Identity(x_dim,x_dim) + A_c * Ts;
    B_sd = Ts*B_c;
    // B_sd = tmp_BT * B_sc;
    // std::cout << "A_sd:"<< std::endl<< A_sd << std::endl;
    // std::cout << "B_sd:"<< std::endl<< B_sd << std::endl;

    // Eigen::MatrixXd Dstate_vector;
    // Dstate_vector.resize(x_dim,1);
    // Dstate_vector(0,0)=vbx*cos(yaw) - sin(yaw)*(vy - (wbz*(rlx + rrx))/2 + Ts*vbx*wbz);
    // Dstate_vector(1,0)=vbx*sin(yaw) + cos(yaw)*(vy - (wbz*(rlx + rrx))/2 + Ts*vbx*wbz);
    // Dstate_vector(2,0)=(Flx + Frx)/M;
    // Dstate_vector(3,0)=vy;
    // Dstate_vector(4,0)=Drlx*wbz + Drrx*wbz - vbx*wbz + (Fly + Fry)/M + (rlx*(Fly*rlx + Fry*rrx + Frx*(D/2 + y) - Flx*(D/2 - y)))/(2*(Ib_zz + 2*Iw_zz)) + (rrx*(Fly*rlx + Fry*rrx + Frx*(D/2 + y) - Flx*(D/2 - y)))/(2*(Ib_zz + 2*Iw_zz));
    // Dstate_vector(5,0)=0;//vbz;
    // Dstate_vector(6,0)=(Flz + Frz - M*g)/M;
    // Dstate_vector(7,0)=0;//wbx;
    // Dstate_vector(8,0)=-(Fly*rlz + Fry*rrz + Frz*(D/2 + y) - Flz*(D/2 - y) - Fly*R_wheel - Fry*R_wheel)/Ib_xx;
    // Dstate_vector(9,0)=0;//wbz;
    // Dstate_vector(10,0)=(Tly + Try + Flx*rlz - Flz*rlx + Frx*rrz - Frz*rrx)/Ib_yy;
    // Dstate_vector(11,0)=wbz;
    // Dstate_vector(12,0)=(Fly*rlx + Fry*rrx + Frx*(D/2 + y) - Flx*(D/2 - y))/(Ib_zz + 2*Iw_zz);
    // Dstate_vector(13,0)=Drlx;
    // Dstate_vector(14,0)=rlx*wbz*wbz - wbz*(2*vy + rlx*wbz - rrx*wbz) - (Flx + Frx)/M + ((D/2 - y)*(Fly*rlx + Fry*rrx + Flx*y + Frx*y - (D*Flx)/2 + (D*Frx)/2))/(Ib_zz + 2*Iw_zz) - (R_wheel*(Tly + Flx*R_wheel))/(Iw_yy + R_wheel*R_wheel*m);
    // Dstate_vector(15,0)=Drrx;
    // Dstate_vector(16,0)=rrx*wbz*wbz - wbz*(2*vy - rlx*wbz + rrx*wbz) - (Flx + Frx)/M - ((D/2 + y)*(Fly*rlx + Fry*rrx + Flx*y + Frx*y - (D*Flx)/2 + (D*Frx)/2))/(Ib_zz + 2*Iw_zz) - (R_wheel*(Try + Frx*R_wheel))/(Iw_yy + R_wheel*R_wheel*m);
    d_sd.resize(x_dim,1);
    d_sd.setZero();
    // d_sd = Ts * Dstate_vector - Ts * A_c * state_point - B_sd * input_point; 
}
void ocp::setWeightLQR(Eigen::DiagonalMatrix<double,Dynamic> Q_diag,Eigen::DiagonalMatrix<double,Dynamic> R_diag){
    Q_s_lqr = Q_diag;
    R_s_lqr = R_diag;
    // Riccati.resize(x_dim,x_dim);
    // Riccati.setZero();
    // solve_riccati(&Riccati,1e-9,2e3);
    // Riccati = Q_s_lqr;

}
void ocp::setWeightMPC(Eigen::DiagonalMatrix<double,Dynamic> Q_diag,Eigen::DiagonalMatrix<double,Dynamic> R_diag){
    Q_s_mpc = Q_diag;
    R_s_mpc = R_diag;
    Riccati.resize(x_dim,x_dim);
    Riccati.setZero();
    // solve_riccati(&Riccati,1e-9,2e3);
    Riccati = Q_s_mpc;

}

void ocp::setHorizon(int horizon){
    if(horizon<2 || horizon >20){
        std::cout << "horizon too short or too long set to 10"<<std::endl;
        this->horizon = 10;
    }
    else{
        this->horizon = horizon;
    }
    // set all qp matrix size
    Aqp.resize(x_dim*horizon, x_dim);
    Aqp.setZero();
    Bqp.resize(x_dim*horizon, u_dim*horizon);
    Bqp.setZero();
    Lqp.resize(x_dim*horizon,x_dim*horizon);
    Lqp_Diag.resize(x_dim*horizon);
    Lqp.setZero();
    Kqp.resize(u_dim*horizon,u_dim*horizon);
    Kqp_Diag.resize(u_dim*horizon);
    Kqp.setZero();
    Hqp.resize(u_dim*horizon, u_dim*horizon);
    Hqp.setZero();
    Gqp.resize(u_dim*horizon, 1);
    Gqp.setZero();
    Dqp.resize(x_dim*horizon,1);
    Dqp.setZero();

    Xdqp.resize(x_dim*horizon, 1);
    Xdqp.setZero();

    Cqp.resize(c_dim*horizon,x_dim);
    LSbqp.resize(c_dim*horizon,1);
    USbqp.resize(c_dim*horizon,1);
    Sbqp.resize(c_dim*horizon,u_dim*horizon);
    // setConstrainMatrix();
}

bool ocp::dlqr_calc(Eigen::MatrixXd *K, double eps ,int max_iter) {
    Eigen::MatrixXd A = A_sd;
    Eigen::MatrixXd B = B_sd;
    // check if dimensions are compatible
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q_s_lqr.rows() != Q_s_lqr.cols() ||
        Q_s_lqr.rows() != A.rows() || R_s_lqr.rows() != R_s_lqr.cols() || R_s_lqr.rows() != B.cols()
        ) {
    ROS_ERROR_STREAM("One or more matrices have incompatible dimensions. Aborting.");
    return false;
    }

    // precompute as much as possible
    Eigen::MatrixXd B_T = B.transpose();
    Eigen::MatrixXd Acal = A;
    Eigen::MatrixXd Acal_T = Acal.transpose();
    Eigen::MatrixXd Qcal = Q_s_lqr;
    Eigen::MatrixXd Rcal = R_s_lqr;

    // initialize P with Q
    Eigen::MatrixXd P = Q_s_lqr;
    Eigen::MatrixXd delta;
    // iterate until P converges
    unsigned int numIterations = 0;
    Eigen::MatrixXd Pold = P;
    while (numIterations<max_iter) {
        numIterations++;

        // compute new P
        P = Acal_T * P * Acal -
            Acal_T * P * B * (Rcal + B_T * P * B).inverse() * B_T * P * Acal + Qcal;

        // update delta
        delta = P - Pold;
        if (delta.lpNorm<Eigen::Infinity>() < eps) {
            ROS_DEBUG_STREAM("Number of iterations until convergence: " << numIterations);
            break;
        }
        Pold = P;
    }

    if(!(numIterations<max_iter)){
        ROS_ERROR_STREAM("Error until " << numIterations <<" iter: " <<"max iter: "<<max_iter<< fabs(delta.lpNorm<Eigen::Infinity>()));
        return false;
    }

    // compute K from P
    *K = (Rcal + B_T * P * B).inverse() * (B_T * P * A);

    return true;
}

void ocp::setQpMatrix()
{
    Eigen::Matrix<double, -1, -1> powerMatsA[this->horizon+1];
    powerMatsA[0].resize(x_dim,x_dim);
    powerMatsA[0].setIdentity();
    for(int i=1; i<this->horizon + 1; i++){
        powerMatsA[i].resize(x_dim,x_dim);
        powerMatsA[i] = A_sd * powerMatsA[i-1];
    }

    for(int r=0;r<this->horizon;r++){
        Aqp.block(x_dim*r, 0, x_dim, x_dim) = powerMatsA[r+1];
        Cqp.block(c_dim*r, 0, c_dim, x_dim) = C_c * powerMatsA[r+1];
        for(int c=0;c<this->horizon;c++){
            if(r >= c){
                int a = r-c;
                Bqp.block(x_dim*r, u_dim*c, x_dim, u_dim) = powerMatsA[a] * B_sd;
                Sbqp.block(c_dim*r, u_dim*c, c_dim, u_dim) = C_c * powerMatsA[a] * B_sd;
            }
        }
        Dqp.block(x_dim*r,0,x_dim,1) = powerMatsA[r]*d_sd;
    }
}

Eigen::MatrixXd ocp::getEquFeedbackMatrix(){
    Eigen::MatrixXd K;
    K = -Hqp.inverse() * Bqp.transpose()*Lqp_Diag*Aqp;
    return K.block(0,0,u_dim,x_dim);
}

Eigen::MatrixXd ocp::setTrajXdqp(Eigen::Matrix<double,Dynamic,1> balance_state,const Regulator & regulator){
    //for test all zero traj
    Eigen::Matrix<double, -1, 1> des_global_state;
    des_global_state.resize(x_dim*horizon,1);

    for (int i = 0; i < horizon; ++i) {

        Eigen::VectorXd des_state = regulator.get_des_state(ros::Time::now()+ros::Duration((i+1)/param.mpc_freq_max));
        Xdqp.block(x_dim * i, 0, x_dim, 1) = des_state - balance_state;
        Xdqp(i*x_dim + 11,0) = yaw_error(des_state(11),balance_state(11));
        des_global_state.block(x_dim * i, 0, x_dim, 1) = des_state;
    }
    // for (int r = 0; r < horizon; ++r) {
    //     Xdqp.block(x_dim * r, 0, x_dim, 1) = powerMatsTraj[r];
    // }
    return des_global_state;
}

Eigen::MatrixXd ocp::getTrajXdqp(){
    return Xdqp;
}

Eigen::MatrixXd ocp::setTrajXdqp(Eigen::Matrix<double,Dynamic,1> balance_state,const Ratio & ratio,Eigen::Matrix<double,17,1> &goal_state){
    
    double des_vx = 0.8*(0.5*ratio.right_v) +0.2*des_vx;
    double des_wz = 0.8*(-0.5*ratio.left_h)+ 0.2*des_wz;
    // double des_z = 0.5*(0.35+0.1*ratio.right_knob)+0.5*des_z;
    // double des_y = 0.2*(-0.05*ratio.right_h) + 0.8*des_y;
    // Eigen::Matrix<double, -1, 1> powerMatsTraj[horizon];
    Eigen::Matrix<double, -1, 1> des_global_state;
    des_global_state.resize(x_dim*horizon,1);
    des_global_state.block(0 , 0, x_dim, 1) = goal_state;
    Xdqp.block(0 , 0, x_dim, 1) = goal_state - balance_state;

    for (int r = 1; r < horizon; ++r) {
        des_global_state.block(x_dim * r, 0, x_dim, 1) = des_global_state.block(x_dim * (r-1), 0, x_dim, 1);
        Xdqp.block(x_dim * r, 0, x_dim, 1) = Xdqp.block(x_dim * (r-1), 0, x_dim, 1);
    }

    return des_global_state;
}

void ocp::setConvexMatrix(){

    Lqp.diagonal() << Q_s_mpc.diagonal().replicate(horizon-1, 1),Eigen::MatrixXd::Zero(x_dim,1);
    Lqp.block(x_dim*(horizon-1),x_dim*(horizon-1),x_dim,x_dim) = Riccati;
    // Lqp.diagonal() << Q_s_mpc.diagonal().replicate(horizon, 1);
    Kqp.diagonal() = R_s_mpc.diagonal().replicate(horizon, 1);

    // use Diagnal matrix for accelerating calculation
    Lqp_Diag.diagonal() = Lqp.diagonal();
    Kqp_Diag.diagonal() = Kqp.diagonal();
    // use sparse matrix
    // Lqp_sparse = Lqp.sparseView();
    // Kqp_sparse = Kqp.sparseView();

    Hqp = Bqp.transpose()*Lqp_Diag*Bqp;
    Hqp.diagonal() += Kqp_Diag.diagonal();//[8*h,8*h]
    Gqp = Bqp.transpose()*Lqp_Diag*(Aqp*X0 + Dqp - Xdqp); //[17*h,8*h]'*[17*h,17*h]*[17*h,1]=[8*h,1]
}

void ocp::setConstrainMatrix(){
    Eigen::MatrixXd lsbqp,usbqp;
    lsbqp.resize(c_dim,1);
    lsbqp << -0.8,-0.1,-1.0;
    usbqp.resize(c_dim,1);
    usbqp << 0.8,0.1,1.0;


    lbqp.resize(u_dim*horizon,1);
    ubqp.resize(u_dim*horizon,1);

    int i=0;
    for(i=0;i<horizon;i++){
        LSbqp.block(i*c_dim,0,c_dim,1) << lsbqp - (Cqp*X0).block(i*c_dim,0,c_dim,1);
        USbqp.block(i*c_dim,0,c_dim,1) << usbqp - (Cqp*X0).block(i*c_dim,0,c_dim,1);

        lbqp.block(u_dim*i,0,u_dim,1) << -13,-15,-30,-10,-13,-15,-30,-10;
        ubqp.block(u_dim*i,0,u_dim,1) << 13,15,30,10,    13,15,30,10;
    }
    // ROS_INFO_STREAM("lqp: "<<lbqp.transpose());
    // ROS_INFO_STREAM("uqp: "<<ubqp.transpose());
}

bool ocp::solve_riccati(Eigen::MatrixXd *Riccati, double eps ,int max_iter){
    Eigen::MatrixXd A = A_sd;
    Eigen::MatrixXd B = B_sd;
    // check if dimensions are compatible
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q_s_lqr.rows() != Q_s_lqr.cols() ||
        Q_s_lqr.rows() != A.rows() || R_s_lqr.rows() != R_s_lqr.cols() || R_s_lqr.rows() != B.cols()
        ) {
    std::cout << "One or more matrices have incompatible dimensions. Aborting."
                << std::endl;
    return false;
    }

    // precompute as much as possible
    Eigen::MatrixXd B_T = B.transpose();
    Eigen::MatrixXd Acal = A;
    Eigen::MatrixXd Acal_T = Acal.transpose();
    Eigen::MatrixXd Qcal = Q_s_lqr;
    Eigen::MatrixXd Rcal = R_s_lqr;

    // initialize P with Q
    Eigen::MatrixXd P = Q_s_lqr;
    Eigen::MatrixXd delta;
    // iterate until P converges
    unsigned int numIterations = 0;
    Eigen::MatrixXd Pold = P;
    while (numIterations<max_iter) {
        numIterations++;

        // compute new P
        P = Acal_T * P * Acal -
            Acal_T * P * B * (Rcal + B_T * P * B).inverse() * B_T * P * Acal + Qcal;

        // update delta
        delta = P - Pold;
        if (delta.lpNorm<Eigen::Infinity>() < eps) {
            std::cout << "Number of iterations until ricatti convergence: " << numIterations
                    << std::endl;
            break;
        }
        Pold = P;
    }

    if(!numIterations<max_iter){
    std::cout << "ricatti Error until " << numIterations <<" iter: " << fabs(delta.lpNorm<Eigen::Infinity>())
                << std::endl;
    }
    *Riccati = P;
    return true;
}

bool ocp::solve_mpc(Eigen::Matrix<double,Dynamic,1> x0,
Eigen::Matrix<double,Dynamic,1> balance_state,
Regulator & regulator,
Eigen::Matrix<double,Dynamic,1> * solution,
Eigen::Matrix<double,Dynamic,1> * Xh,
bool verbose){
    Timer init_time;
    this->X0 = x0 - balance_state;
    this->X0(5) = yaw_error(x0(5),balance_state(5));

    // setTrajXdqp(balance_state,regulator); // set in controller, before solve_mpc
    if(!malloced){
        H_qpoases = (qpOASES::real_t*)malloc(u_dim*horizon*u_dim*horizon*sizeof(qpOASES::real_t));
        g_qpoases = (qpOASES::real_t*)malloc(u_dim*horizon*sizeof(qpOASES::real_t));
        Sb_qpoases = (qpOASES::real_t*)malloc(u_dim*horizon*c_dim*horizon*sizeof(qpOASES::real_t));
        lSb_qpoases = (qpOASES::real_t*)malloc(c_dim*horizon*sizeof(qpOASES::real_t));
        uSb_qpoases = (qpOASES::real_t*)malloc(c_dim*horizon*sizeof(qpOASES::real_t));
        lb_qpoases = (qpOASES::real_t*)malloc(u_dim*horizon*sizeof(qpOASES::real_t));
        ub_qpoases = (qpOASES::real_t*)malloc(u_dim*horizon*sizeof(qpOASES::real_t));
        q_soln = (qpOASES::real_t*)malloc(u_dim*horizon*sizeof(qpOASES::real_t));
        malloced = true;
    }

    setQpMatrix(); // calculate Aqp and Bqp

    if(verbose) std::cout << "Aqp  Bqp matrix set time "<< init_time.getMs()<<" ms" << std::endl;
    init_time.start();

    setConvexMatrix();// calculate  Lqp Kqp Hqp Gqp
    setConstrainMatrix(); // calculate LSbqp USbqp Sbqp
    if(verbose) std::cout << "Hqp Gqp Sbqp USbqp LSbqp matrix set time "<< init_time.getMs()<<" ms" << std::endl;
    init_time.start();

    matrix_to_real(H_qpoases, Hqp, u_dim*horizon, u_dim*horizon);
    matrix_to_real(g_qpoases, Gqp, u_dim*horizon, 1);
    matrix_to_real(Sb_qpoases, Sbqp, c_dim*horizon, u_dim*horizon);
    matrix_to_real(uSb_qpoases, USbqp, c_dim*horizon, 1);
    matrix_to_real(lSb_qpoases, LSbqp, c_dim*horizon, 1);
    matrix_to_real(lb_qpoases, lbqp, u_dim*horizon, 1);
    matrix_to_real(ub_qpoases, ubqp, u_dim*horizon, 1);
    

    int num_constraints = c_dim*horizon;//fmat.rows();
    int num_variables = u_dim*horizon;//fmat.cols();
    qpOASES::int_t nWSR = 1000;//200
    // qpOASES::QProblem problem_red(num_variables, num_constraints);
    qpOASES::QProblem problem_red(num_variables,num_constraints);
    qpOASES::Options op;
    op.setToMPC();
    // op.setToReliable();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);

    int rval2 = problem_red.init(H_qpoases, g_qpoases, Sb_qpoases, lb_qpoases, ub_qpoases, lSb_qpoases, uSb_qpoases, nWSR);
    // int rval2 = problem_red.init(H_qpoases, g_qpoases, lb_qpoases, ub_qpoases, nWSR);
    // int rval2 = problem_red.init(H_qpoases, g_qpoases, NULL, NULL, NULL, NULL, NULL, nWSR);
    (void) rval2;
     if(rval2 != qpOASES::SUCCESSFUL_RETURN){
         std::cout<<"failed to init QP!"<<std::endl;
          if(rval2 == qpOASES::RET_MAX_NWSR_REACHED) std::cout<<"RET_MAX_NWSR_REACHED"<<std::endl;
          else if (rval2 == qpOASES::RET_INIT_FAILED) std::cout<<"RET_INIT_FAILED"<<std::endl;
          else if(rval2 == qpOASES::RET_INIT_FAILED_HOTSTART) std::cout<<"QP Cannot Be Solved!"<<std::endl;
          std::cout<<"rval2: "<<rval2<<std::endl;
          return false;
     }
    int rval = problem_red.getPrimalSolution(q_soln);
    if(rval != qpOASES::SUCCESSFUL_RETURN){
        std::cout<<"failed to solve!"<<std::endl;
        return false;
    }


    real_to_matrix(U_soln, q_soln, u_dim*horizon);
    // for(int i= 0;i<5;i++){
    //     ROS_INFO_STREAM("i:"<<i<<"  -> "<< U_soln.block(i*u_dim,0,u_dim,1).transpose());
    // }
    *solution = U_soln.block(0,0,u_dim,1);
    if(verbose) std::cout << "solve time "<< init_time.getMs()<<" ms" << std::endl;

    *Xh = Aqp * X0 + Bqp * U_soln;
    for (int i= 0;i<horizon;i++){
        Xh->block(i*x_dim,0,x_dim,1) += balance_state;
    }
    
    return true;
}

void ocp::free_qp(){
    if(malloced){
        free(H_qpoases);
        free(g_qpoases);
        free(Sb_qpoases);
        free(lSb_qpoases);
        free(uSb_qpoases);
        free(lb_qpoases);
        free(ub_qpoases);
        free(q_soln);
        H_qpoases = NULL;
        g_qpoases = NULL;
        Sb_qpoases = NULL;
        lSb_qpoases = NULL;
        uSb_qpoases = NULL;
        lb_qpoases = NULL;
        ub_qpoases = NULL;
        q_soln = NULL;
        malloced = false;
    }
}

void ocp::matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int rows, int cols)
{
  int a = 0;
  for(int r = 0; r < rows; r++)
  {
    for(int c = 0; c < cols; c++)
    {
      dst[a] = (double)src(r,c);
      a++;
    }
  }
}

void ocp::real_to_matrix(Eigen::Matrix<double,Dynamic,1>& dst, qpOASES::real_t* src, int n_items)
{
    dst.resize(n_items,1);
    dst.setZero();
    for(int i=0;i<n_items;i++)
        dst(i) = (double)src[i];
}