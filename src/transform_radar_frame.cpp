#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <iostream>

using namespace std;

class RadarTF
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string target_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

public:
  RadarTF():
  tf_listener_(tf_buffer_)
  {
  ros::NodeHandle n;
  sub_ = nh_.subscribe("rzi_filt_out", 1, &RadarTF::callback, this);
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("rzi_filt_out/transformed", 1);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr &radar_pcl_msg);

};

void RadarTF::callback(const sensor_msgs::PointCloud2ConstPtr &radar_pcl_msg){

    geometry_msgs::TransformStamped transformStamped;
    target_frame_ = "oflow_frame";
    try{
    transformStamped = tf_buffer_.lookupTransform("base_radar_link", target_frame_, ros::Time(0));
    sensor_msgs::PointCloud2 cloud_out;
      tf2::doTransform(*radar_pcl_msg, cloud_out, transformStamped);
      cloud_out.header.frame_id = target_frame_;
      cloud_out.header.stamp = ros::Time::now();
      pub_.publish(cloud_out);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

  //cout << transformStamped.transform.translation.x;
}


int main(int argc, char** argv){
 ros::init(argc, argv, "transform_radar_frame_node");
 RadarTF radartf;

 ros::spin();
}
