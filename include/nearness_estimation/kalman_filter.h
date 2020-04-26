#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
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
        void odomCb(const nav_msgs::OdometryConstPtr &odom_msg);
        void predict();
        void removeOutliers();
        void kalmanGain();
        void update();
        void filterData();
        void publishLaser();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::string node_name_{"node_name"};

        ros::Subscriber sub_radar_vel_;
        ros::Subscriber sub_radar_scan_;
        ros::Subscriber sub_imu_;
        ros::Subscriber sub_tang_flow_;
        ros::Subscriber sub_odom_;

        ros::Publisher pub_mu_;
        ros::Publisher pub_mu_unfilt_;
        ros::Publisher pub_mu_norad_;

        ros::Publisher pub_laser_;
        ros::Time last_timestamp_;

        int N_;
        int Nrad_;
        int Ndy_;
        int k_;
        float u_;
        float v_;
        float r_;
        float last_r_;
        float last_u_;
        float last_v_;
        float pos_x_;
        float pos_y_;
        float last_pos_x_;
        float last_pos_y_;
        double q_;
        double r_oflow_;
        double r_rad_;
        double yaw_;
        double dt_;
        double thresh_;
        double min_vel_;
        double f_smooth_;
        bool init_;
        bool flag_imu_;
        bool flag_oflow_;
        bool flag_radar_;
        bool flag_vel_;
        bool flag_odom_;

        VectorXd state_;
        VectorXd state0_;
        VectorXd state_pred_;
        VectorXd state_future_;
        VectorXd last_state_;
        VectorXd state_nofilt_;
        VectorXd state_norad_;
        VectorXd y_rad_;
        VectorXd y_;
        VectorXd oflow_;
        VectorXd last_oflow_;
        VectorXf gamma_vector_;
        VectorXd f_;
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

};


// Class for Lowess filter
class TestContainer
{

  std::vector<double> data_;

public:

  typedef std::vector<double>::iterator iterator;

  TestContainer() {}
  explicit TestContainer(int n)
  {
    data_.resize(n);
  }

  inline size_t size() const
  {
    return data_.size();
  }

  inline double& operator[](size_t n)
  {
    return data_[n];
  }

  const inline double& operator[](size_t n) const
  {
    return data_[n];
  }

  inline void push_back(double x)
  {
    return data_.push_back(x);
  }

  inline std::vector<double>::iterator begin()
  {
    return data_.begin();
  }

  inline std::vector<double>::iterator end()
  {
    return data_.end();
  }

};


#endif
