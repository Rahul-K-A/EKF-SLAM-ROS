#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>
#include <stack>

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;


class ExtendedKalmanFilter
{
    public:

        bool Initialised;
        ExtendedKalmanFilter();
        void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void Init(int argc, char* argv[]);
        void Predict();
        void Update();
        void PublishKalmanVals();
        double WrapAngle(double Angle);
        ~ExtendedKalmanFilter();
    
    private:
        VectorXd X_prev;
        VectorXd X;
        MatrixXd P;
        MatrixXd P_prev;
        MatrixXd F;
        MatrixXd Q;
        MatrixXd H;
        MatrixXd R;
        ros::Time PrevTime;
        bool IsInitialised;
        bool RecievedReading;
        ros::NodeHandle* n;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        ros::Time CurrentTime;
        struct OdometryInfo{
            VectorXd Z;      
            double angVel;      
        } CurrentReading;
        nav_msgs::Odometry current_reading;
        geometry_msgs::Pose2D kalman_vals;


};