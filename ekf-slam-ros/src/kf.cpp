#include "kalman_linear/kf.h"
   
KalmanFilter::KalmanFilter()
{
    X_prev = VectorXd::Zero(6);
    X = VectorXd::Zero(6);
    P = MatrixXd::Zero(6,6);
    P_prev = MatrixXd::Zero(6,6);
    F = MatrixXd::Identity(6,6);
    L = MatrixXd::Identity(6,3);
    Q = MatrixXd::Identity(3,3);
    CurrentReading.Z = VectorXd::Zero(6);
    H = MatrixXd::Identity(6,6);
    R = MatrixXd::Zero(6,6);
    R(0,0) =  1e-05;
    R(1,1) = 1e-05;
    R(2,2) = 1e-05;
    IsInitialised = false;
}

