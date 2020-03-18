#include <nearness_estimation/nearness_estimation.h>
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "simple_nearness_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("simple_nearness_node");
    ros::NodeHandle nh_private("~");
    NearnessEstimation nearness_estimation(nh, nh_private);
    ros::spin();
}
