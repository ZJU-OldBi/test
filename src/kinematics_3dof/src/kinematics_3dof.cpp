#include "kinematics_3dof.h"
#include <cmath>

namespace Kinematics
{
    Eigen::Vector3d ik(const Eigen::Vector3d& transform, double L1, double L2,double L3){
        double x = transform(0);
        double y = transform(1);
        double z = transform(2);
        double D = sqrt(x*x+y*y+z*z-L3*L3);
        if(D>L1+L2-1e-5){
            x *= (L1+L2-1e-5)/D;
            y *= (L1+L2-1e-5)/D;
            z *= (L1+L2-1e-5)/D;
            D = sqrt(x*x+y*y+z*z-L3*L3);
        }
        double H = sqrt(y*y+z*z-L3*L3);
        double q3 = atan2(H,L3)-atan2(fabs(z),y);
        
        double q2 = 3.1415926 - acos((L1*L1+L2*L2-D*D)/(2*L1*L2)); // 0 < q2 < PI
        
        double x_ = H;
        double y_ = x;
        double q1 = atan(y_/x_) - acos((L1*L1+D*D-L2*L2)/(2*L1*D));

        return Eigen::Vector3d(q3,-q1,-q2);
    }

    Eigen::Vector3d k(const Eigen::Vector3d& joint_angles, double L1, double L2,double L3){
        double q3 =joint_angles(0);
        double q1 =joint_angles(1);
        double q2 =joint_angles(2);
        double x_ = L2*cos(q1+q2)+L1*cos(q1);
        double y_ = L2*sin(-q1-q2)+L1*sin(-q1);

        return Eigen::Vector3d(y_,x_*sin(q3)+L3*cos(q3),-x_*cos(q3)+L3*sin(q3));
    }

    Eigen::Matrix3d Jp(const Eigen::Vector3d& joint_angles,double L1,double L2,double L3){
        Eigen::Matrix3d jacobian;
        double q3 =joint_angles(0);
        double q1 =joint_angles(1);
        double q2 =joint_angles(2);
        double c1 = cos(q1);
        double s1 = sin(q1);
        double c3 = cos(q3);
        double s3 = sin(q3);
        double c12 = cos(q1+q2);
        double s12 = sin(q1+q2);
        jacobian <<            0           ,   -L2*c12-L1*c1   ,  -L2*c12  ,
                    c3*(L2*c12+L1*c1)-L3*s3,-s3*(L2*s12+L1*s1),-L2*s12*s3,
                    s3*(L2*c12+L1*c1)+L3*c3, c3*(L2*s12+L1*s1), L2*s12*c3;
        return jacobian;
    }

    Eigen::Matrix3d Jr(const Eigen::Vector3d& joint_angles){
        Eigen::Matrix3d jacobian;
        double q3 =joint_angles(0);
        jacobian << 1,        0,          0,
                    0,  cos(q3),    cos(q3),
                    0,  sin(q3),    sin(q3);
        return jacobian;                    
    }

} // namespace Kinematics

