#include <depth_estimation/depth_estimation.h>
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "simple_depth_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("simple_depth_node");
    ros::NodeHandle nh_private("~");
    DepthEstimation depth_estimation(nh, nh_private);
    ros::spin();
}
