#include "kalman_linear/lkf.h"
#include <tf/tf.h>
#include <random>
#include <cmath>
#include <iostream>


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
    R(3,3) = 1e-05;
    IsInitialised = false;
}


double KalmanFilter::WrapAngle(double Angle)
{
    if (Angle > M_PI)
    {
        double diff = Angle - double(M_PI);
        return -M_PI + diff;

    }
    else if (Angle < -M_PI)
    {
        float diff =  Angle + double(M_PI);
        return M_PI - diff;
    }
}

void KalmanFilter::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double px,py,theta,vx,vy,omega,pitch,roll;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w)).getEulerYPR(theta,pitch,roll);
    vx = msg->twist.twist.linear.x;
    vx = msg->twist.twist.linear.y;
    omega = msg->twist.twist.angular.z;
    CurrentReading.Z<<px,py,theta,vx,vy,omega;
}

void KalmanFilter::Update()
{
    // ROS_INFO("Update");
    if (IsInitialised)
    {
        MatrixXd Y_hat = CurrentReading.Z - H*X;
        MatrixXd S = H*P*H.transpose() + R;
        MatrixXd K = P*H.transpose()*S.inverse();
        X = X + K*Y_hat;
        P =(MatrixXd::Zero(6,6) - K*H)*P;
        P_prev = P;
        X_prev = X;
    }
    else{
        X_prev = CurrentReading.Z;
        P_prev = R;
        IsInitialised = true;
    }
    

}


void KalmanFilter::Predict()
{
    if(!IsInitialised){
        return;
    }
    // ROS_INFO("Predict");
    ros::Time current_time = ros::Time::now();
    double dt =  double(current_time.sec - PrevTime.sec) + double(current_time.nsec - PrevTime.nsec)*1e-9;
    F(0,3) = dt;
    F(1,4) = dt;
    F(2,5) = dt;
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{0, 1};
    Q(0,0)= d(gen);
    Q(1,1)= d(gen);
    Q(2,2)= d(gen);
    double L_coeff = 0.5f * dt *dt;
    L(0,0) = L_coeff;
    L(1,1) = L_coeff;
    L(2,2) = L_coeff;
    L(3,0) = dt;
    L(4,1) = dt;
    L(5,2) = dt;
    X = F*X_prev;
    P = F*P_prev*F.transpose() + L*Q*L.transpose();
}

void KalmanFilter::PublishKalmanVals()
{
    kalman_vals.x = X(0);
    kalman_vals.y = X(1);
    kalman_vals.theta = X(2);
    pub_.publish(kalman_vals);
}

void KalmanFilter::Init(int argc, char* argv[])
{
    ros::init(argc,argv,"Kalman_Filter");
    n = new ros::NodeHandle;
    sub_ = n->subscribe("/odom", 1000, &KalmanFilter::OdomCallback,this);
    pub_ = n->advertise<geometry_msgs::Pose2D>("/kalman_vals",1000);
    PrevTime = ros::Time::now();
    ros::spinOnce();
}

KalmanFilter::~KalmanFilter()
{
delete n;
n = nullptr;
}

int main(int argc, char* argv[])
{
    KalmanFilter KF;
    KF.Init(argc,argv);
    ros::Rate loop_rate(12);
    while(ros::ok())
    {
        KF.Predict();
        KF.Update();
        KF.PublishKalmanVals();
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}