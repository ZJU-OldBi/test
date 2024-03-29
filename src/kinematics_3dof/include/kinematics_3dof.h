#pragma once
#include <eigen3/Eigen/Dense>
namespace Kinematics{
    /*
    input:foot endpoint position relative to body frame
    output:value of each 3 joint angles(hip,leg1,leg2)
    */
    Eigen::Vector3d ik(const Eigen::Vector3d& transform, double L1, double L2,double L3);

    /*
    input:value of each 3 joint angles(hip,leg1,leg2)
    output:foot endpoint position relative to body frame
    */
    Eigen::Vector3d k(const Eigen::Vector3d& joint_angles, double L1, double L2,double L3);

    /*
    input:value of each 3 joint angles(hip,leg1,leg2)
    output:foot endpoint speed Jacobian relative to body frame
    */
    Eigen::Matrix3d Jp(const Eigen::Vector3d& joint_angles,double L1,double L2,double L3);

    /*
    input:value of each 3 joint angles(hip,leg1,leg2)
    output:foot endpoint rotaion Jacobian relative to body frame
    */
    Eigen::Matrix3d Jr(const Eigen::Vector3d& joint_angles);
}

