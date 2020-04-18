#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class KalmanFilter{
    public:
        KalmanFilter(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
        ~KalmanFilter() = default;

        void init();

        // FUNCTIONS
        void radarvelCb(const geometry_msgs::TwistWithCovarianceStampedConstPtr &radar_vel_msg);
        void radarscanCb(const sensor_msgs::LaserScanConstPtr &radar_scan_msg);
        void imuCb(const sensor_msgs::ImuConstPtr &imu_msg);
        void oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg);
        void predict();
        void kalmanGain();
        void update();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::string node_name_{"node_name"};

        ros::Subscriber sub_radar_vel_;
        ros::Subscriber sub_radar_scan_;
        ros::Subscriber sub_imu_;
        ros::Subscriber sub_tang_flow_;

        ros::Publisher pub_mu_;
        //ros::Publisher pub_laser_;
        ros::Time last_timestamp_;

        int N_;
        int Nrad_;
        int Ndy_;
        float u_;
        float v_;
        float r_;
        float last_r_;
        float last_u_;
        float last_v_;
        double q_;
        double r_oflow_;
        double r_rad_;
        double dt_;
        bool init_;
        bool flag_imu_;
        bool flag_oflow_;
        bool flag_radar_;
        bool flag_vel_;
      //  bool publish_laser_;

        MatrixXd identity_;
        MatrixXd F_;
        MatrixXd Q_;
        MatrixXd R_oflow_;
        MatrixXd R_rad_;
        MatrixXd R_;
        MatrixXd P0_;
        MatrixXd P_update_;
        MatrixXd last_P_;
        MatrixXd P_pred_;
        MatrixXd H_;
        MatrixXd K_;
        MatrixXd KH_;
        VectorXd state0_;
        VectorXd y_rad_;
        VectorXd y_;
        VectorXd oflow_;
        VectorXd last_oflow_;
        VectorXf gamma_vector_;
        VectorXd state_;
        //VectorXf mu_vector_;

};


#endif
