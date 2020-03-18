#ifndef NEARNESS_ESTIMATION_H
#define NEARNESS_ESTIMATION_H

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

class NearnessEstimation{
    public:
        NearnessEstimation(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
        ~NearnessEstimation() = default;

        void init();

        // FUNCTIONS
        void radarCb(const geometry_msgs::TwistWithCovarianceStampedConstPtr &radar_msg);
        void imuCb(const sensor_msgs::ImuConstPtr &imu_msg);
        void oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::string node_name_{"node_name"};

        ros::Subscriber sub_state_;
        ros::Subscriber sub_imu_;
        ros::Subscriber sub_tang_flow_;
        ros::Publisher pub_mu_;
        ros::Publisher pub_laser_;
        ros::Time imu_time_;

        int num_ring_points_;
        VectorXf gamma_vector_;
        VectorXf mu_vector_;
        VectorXf ave_tang_flow_;

        // Radar velocity
        double psi_;
        float u_;
        float v_;
        float r_;
        bool init_;
        bool publish_laser_;

};


#endif
