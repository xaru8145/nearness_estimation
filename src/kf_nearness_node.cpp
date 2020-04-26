#include <nearness_estimation/kalman_filter.h>
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "kalman_filter_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("kalman_filter_node");
    ros::NodeHandle nh_private("~");
    KalmanFilter kalman_filter(nh, nh_private);
    ros::spin();
}
