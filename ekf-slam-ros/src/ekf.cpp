#include "kalman_linear/ekf.h"
#include <tf/tf.h>
#include <random>
#include <cmath>
#include <iostream>


ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    //State: Px,Py,Theta,V
    X_prev = VectorXd::Zero(4);
    X = VectorXd::Zero(4);
    P = MatrixXd::Zero(4,4);
    P_prev = MatrixXd::Zero(4,4);
    F = MatrixXd::Identity(4,4);
    Q = MatrixXd::Identity(4,4);
    CurrentReading.Z = VectorXd::Zero(4);
    H = MatrixXd::Identity(4,4);
    R = MatrixXd::Zero(4,4);
    R(0,0) =  1e-05;
    R(1,1) = 1e-05;
    R(2,2) = 1e-05;
    R(3,3) = 1e-05;
    IsInitialised = false;
    RecievedReading = false;

}

double ExtendedKalmanFilter::WrapAngle(double Angle)
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
    else{
        return Angle;
    }
}


void ExtendedKalmanFilter::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double px,py,theta,vx,vy,omega,pitch,roll;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w)).getEulerYPR(theta,pitch,roll);
    vx = msg->twist.twist.linear.x;
    vx = msg->twist.twist.linear.y;
    double V = sqrt(vx*vx + vy*vy);
    CurrentReading.angVel = msg->twist.twist.angular.z;
    CurrentReading.Z<<px,py,theta,V;
    RecievedReading = true;

}

void ExtendedKalmanFilter::Update()
{
    ROS_INFO("Update");
    if (IsInitialised)
    {
        MatrixXd Y_hat = CurrentReading.Z - X;
        ROS_INFO("Innovation");
        MatrixXd S = H*P*H.transpose() + R;
        ROS_INFO("Kalman Gain");
        MatrixXd K = P*H.transpose()* S.inverse();
        X = X + K*Y_hat;
        P =(MatrixXd::Zero(4,4) - K*H)*P;
        P_prev = P;
        X_prev = X;
    }
    else{
        if(RecievedReading)
        {
            X_prev = CurrentReading.Z;
            P_prev = R;
            IsInitialised = true;
        }
    }
    

}


void ExtendedKalmanFilter::Predict()
{
    if(!IsInitialised){
        return;
    }
    ROS_INFO("Predict");
    ros::Time current_time = ros::Time::now();
    double dt =  double(current_time.sec - PrevTime.sec) + double(current_time.nsec - PrevTime.nsec)*1e-9;
    X(0) = X_prev(0) + dt * X_prev(3) * cos(X_prev(2));
    X(1) = X_prev(1) + dt * X_prev(3) * sin(X_prev(2));
    X(2) = WrapAngle( X_prev(2) + dt * CurrentReading.angVel );
    X(3) = X_prev(3); 


    F(0,2) = -dt * sin(X_prev(2)) * X_prev(3);
    F(0,3) =  dt * cos(X_prev(2));
    F(1,2) =  dt * cos(X_prev(2)) * X_prev(3);
    F(1,3) =  dt * sin(X_prev(2));

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<double> d{0, 1};

    Q(0,0)= dt * dt * d(gen);
    Q(1,1)= dt * dt * d(gen);
    Q(2,2)= dt * dt * d(gen);
    Q(3,3)= dt * dt * d(gen);
    P = F * P_prev * F.transpose() + Q;
}

void ExtendedKalmanFilter::PublishKalmanVals()
{
    kalman_vals.x = X(0);
    kalman_vals.y = X(1);
    kalman_vals.theta = X(2);
    pub_.publish(kalman_vals);
}

void ExtendedKalmanFilter::Init(int argc, char* argv[])
{
    ros::init(argc,argv,"Kalman_Filter");
    n = new ros::NodeHandle;
    sub_ = n->subscribe("/odom", 1000, &ExtendedKalmanFilter::OdomCallback,this);
    pub_ = n->advertise<geometry_msgs::Pose2D>("/kalman_vals",1000);
    PrevTime = ros::Time::now();
    ros::spinOnce();
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
delete n;
n = nullptr;
}

int main(int argc, char* argv[])
{
    ExtendedKalmanFilter KF;
    KF.Init(argc,argv);
    ros::Rate loop_rate(15);
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