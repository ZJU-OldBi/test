#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "qpOASES.hpp"
#include "OD_Param.h"
#include "OD_Observer.h"
#include "OD_Regulator.h"
using Eigen::Dynamic;

class ocp
{
private:
    int x_dim,u_dim,c_dim;
    Eigen::MatrixXd A_c;
    Eigen::MatrixXd B_c;
    Eigen::MatrixXd C_c;
    Eigen::MatrixXd D_c;
    Eigen::MatrixXd d_sd;
    Eigen::MatrixXd A_sc,A_sd;
    Eigen::MatrixXd B_sc,B_sd;
    Eigen::DiagonalMatrix<double,Dynamic> Q_s_lqr,Q_s_mpc;
    Eigen::DiagonalMatrix<double,Dynamic> R_s_lqr,R_s_mpc;
    WLParam param;

    int horizon;
    Eigen::Matrix<double,Dynamic,1> X0;
    Eigen::MatrixXd Lqp;
    Eigen::MatrixXd Kqp;
    Eigen::SparseMatrix<double> Lqp_sparse,Kqp_sparse;
    Eigen::DiagonalMatrix<double, Dynamic> Lqp_Diag,Kqp_Diag;
    Eigen::MatrixXd Riccati;
    Eigen::MatrixXd Aqp; // [A;A^2 ...;A^N]
    Eigen::MatrixXd Bqp;
    Eigen::MatrixXd Cqp; // [CA;CA^2 ...;CA^N]
    Eigen::Matrix<double,Dynamic,1> Dqp;
    Eigen::Matrix<double,Dynamic,1> Xdqp;
    Eigen::MatrixXd LSbqp, USbqp, Sbqp, lbqp, ubqp;
    Eigen::MatrixXd Hqp;
    Eigen::Matrix<double,Dynamic, 1> Gqp;
    Eigen::Matrix<double,Dynamic,1> U_soln;

    qpOASES::real_t* H_qpoases;
    qpOASES::real_t* g_qpoases;
    qpOASES::real_t* Sb_qpoases;
    qpOASES::real_t* lSb_qpoases;
    qpOASES::real_t* uSb_qpoases;
    qpOASES::real_t* lb_qpoases;
    qpOASES::real_t* ub_qpoases;

    qpOASES::real_t* q_soln;

    bool malloced;

    void setQpMatrix();
    void setConvexMatrix();
    
public:
    ocp(WLParam param,int state_dim,int input_dim,int constrain_dim);
    ~ocp();
    bool dlqr_calc(Eigen::MatrixXd *K, double eps = 1e-5,int max_iter = 10000);
    void setStateMatrix(double rlz_,double rrz_,double Ts);
    void setStateMatrix(Observer obser,double Ts,Eigen::MatrixXd state_point,Eigen::MatrixXd input_point);
    void setWeightLQR(Eigen::DiagonalMatrix<double,Dynamic> Q_diag,Eigen::DiagonalMatrix<double,Dynamic> R_diag);
    void setWeightMPC(Eigen::DiagonalMatrix<double,Dynamic> Q_diag,Eigen::DiagonalMatrix<double,Dynamic> R_diag);
    void setHorizon(int horizon);
    Eigen::MatrixXd getEquFeedbackMatrix();
    bool solve_mpc(Eigen::Matrix<double,Dynamic,1> x0,Eigen::Matrix<double,Dynamic,1> balance_state,Regulator & regulator,Eigen::Matrix<double,Dynamic,1> * solution, Eigen::Matrix<double,Dynamic,1> * Xh,bool verbose);
    bool solve_riccati(Eigen::MatrixXd *P, double eps ,int max_iter);
    void setConstrainMatrix();
    void free_qp();

    void matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int rows, int cols);
    void real_to_matrix(Eigen::Matrix<double,Dynamic,1>& dst, qpOASES::real_t* src, int n_items);
    Eigen::MatrixXd setTrajXdqp(Eigen::Matrix<double,Dynamic,1> balance_state,const Regulator & regulator);
    Eigen::MatrixXd setTrajXdqp(Eigen::Matrix<double,Dynamic,1> balance_state,const Ratio & ratio,Eigen::Matrix<double,17,1> &goal_state);
    Eigen::MatrixXd getTrajXdqp();
};


