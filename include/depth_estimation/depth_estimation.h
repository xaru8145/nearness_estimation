#ifndef DEPTH_ESTIMATION_H
#define DEPTH_ESTIMATION_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class DepthEstimation{
    public:
        DepthEstimation(const ros::NodeHandle &node_handle,
                          const ros::NodeHandle &private_node_handle);
        ~DepthEstimation() = default;

        void init();

        // FUNCTIONS
        void stateCb(const geometry_msgs::TwistStampedConstPtr &state_msg);
        void oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg);
        //void depthCb();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::string node_name_{"node_name"};


        ros::Subscriber sub_state_;
        ros::Subscriber sub_tang_flow_;
        ros::Publisher pub_depth_;

        int num_ring_points_;
        VectorXf gamma_vector_;
        VectorXf depth_vector_;
        VectorXf ave_tang_flow_;


        // Mocap state
        double phi_;
        double theta_;
        double psi_;
        float u_;
        float v_;
        float r_;
        bool init_;

        ros::Time image_timestamp_;
        ros::Time last_image_timestamp_;

};


#endif