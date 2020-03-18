#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>

ros::Publisher laser_pub;
void laserCallback(const sensor_msgs::LaserScan& msg)
{
  sensor_msgs::LaserScan laser_msg;
  imu_msg = msg;
  imu_msg.orientation_covariance[0] = 0.00001;
  imu_msg.orientation_covariance[4] = 0.00001;
  imu_msg.orientation_covariance[8] = 0.00001;

  imu_pub.publish(imu_msg);
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "oflow_laserscan");
ros::NodeHandle n;
ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imuCallback);
imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_added_cov", 1000);
ros::spin();

return 0;
}
