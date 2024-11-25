#pragma once

// Include necessary libraries
#include <vector>
#include <iostream>
#include <memory>

#include "eigen3/Eigen/Eigen"
#include "osqp/osqp.h"

class MPC
{
public:
    MPC(int N, double dT, double maxWheelSpeed, double wheelWidth, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd Qf);
    ~MPC();

    void setTrackReference(
        const Eigen::VectorXd& x, 
        const Eigen::VectorXd& y,
        const Eigen::VectorXd& theta,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& w
    );
    bool update(
        const Eigen::VectorXd& state,
        Eigen::VectorXd& control
    );
    void abort();

public:
    double dT;
    int N;

    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;

    Eigen::MatrixXd Q;  //3,3
    Eigen::MatrixXd R;  //2,2
    Eigen::MatrixXd Qf; //3,3

    Eigen::MatrixXd A_hat; //3,3
    Eigen::MatrixXd B_hat; //3,2
    Eigen::MatrixXd A_ba;  //3N,3
    Eigen::MatrixXd B_ba;  //3N,2N
    std::vector<Eigen::MatrixXd> A_hat_power;

    Eigen::VectorXd x_ref;
    Eigen::VectorXd y_ref;
    Eigen::VectorXd theta_ref;
    Eigen::VectorXd v_ref;
    Eigen::VectorXd w_ref;

    Eigen::VectorXd x_dummy;
    Eigen::VectorXd y_dummy;
    Eigen::VectorXd theta_dummy;
    Eigen::VectorXd v_dummy;
    Eigen::VectorXd w_dummy;

    Eigen::VectorXd x_err_ref;
    Eigen::VectorXd y_err_ref;
    Eigen::VectorXd theta_err_ref;

    int current_iter;

    bool enableControl;

    void setup(const Eigen::VectorXd &state);

    bool createOsqpSparseMatrix(
        const Eigen::MatrixXd& matrix,
        csc *& osqp_matrix
    );
private:
    
};

