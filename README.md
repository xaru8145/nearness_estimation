# Nearness Estimation

This repository estimates 2D nearness (inverse of the distance) using a Kalman Filter to fuse optic flow and radar measurements.

The Kalman Filter works at **10 Hz**, which is the slowest sensor rate (radar). All the matrix operations are done manually using a single for loop. We can do this due to the fact that we know the shape of all matrices before hand. This allows us to run the filter real-time.

The **optic flow** data is planar and tangential and is computed from an image distorted by a parabolic mirror. It uses Lucas-Kanade pyramidal algorithm and is filtered through a low-pass filter. Note that the quality of optic flow measurements depend on the texture of the environment.

The **radar** data is filtered by its intensity, range and height. The height filtering strictly depends on the radar location in the robot. For instance, if you do not want to sense the floor, you will need to filter the altitude depending on the height of the radar. Radar data can read through walls and output some outliers. We will implement an outlier removal after converting the 3D pointcloud data into a 2D laserscan. This allows us to handle better the outliers. We remove them by projecting the previous nearness estimate with the robot dynamics into the next time step. This relies on the fact that radar outliers do not appear twice.

A [Lowess](https://github.com/hroest/CppLowess) filter is used to smooth out the data. The algorithm performs locally-weighted polynomial regression in two dimensions.




## Installation
The Nearness Estimation package depends on the following libraries and additional packages.
#### Libraries
* Eigen3
#### ROS Packages
* [optic_flow_node](https://github.com/mohradza/optic_flow_node/tree/dev/xavi) to compute optic flow from images from images.  
* [usb_cam](https://github.com/ros-drivers/usb_cam) as the camera driver.
* [pointcloud_to_laserscan](https://github.com/xaru8145/pointcloud_to_laserscan) to convert radar pcl to laser scan. Used to convert the estimated nearness from laser scan into pcl and create  an octomap.
* [octomap_mapping](https://github.com/OctoMap/octomap_mapping) to create an octomap from pcl data and odometry.
* [robot_localization](https://github.com/xaru8145/robot_localization/tree/oflow-jackal) to estimate odometry from radar velocity and IMU orientation.




## Nodes
### kf_nearness_node
This node uses a Kalman Filter to fuse data and is thoroughly described above.
```bash
roslaunch nearness_estimation kf_nearness.launch
```
### simple_nearness_node
This is a simple version that uses the motion parallax equations to derive the nearness from optic flow measurements. It runs at 30 Hz which is the publishing rate of the measured optic flow. It does not use radar.
```bash
roslaunch nearness_estimation simple_nearness.launch
```



## Kalman Filter Node

#### Subscribed topics

* **/mmWaveDataHdl/velocity** ([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))

* **/radar_scan** ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))

* **/imu/data_added_cov** ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))

* **/optic_flow_node/tang_optic_flow/filtered** ([std_msgs/Float32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html))

* **/odometry/filtered** ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))


#### Published topics
* **/kalman_filter_node/nearness** ([std_msgs/Float64MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html)): estimated nearness

* **/kalman_filter_node/nearness/unfiltered** ([std_msgs/Float64MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html)): estimated nearness without the Lowess filter.

* **/kalman_filter_node/nearness/no_radar** ([std_msgs/Float64MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html)):nearness derived from optic flow, no radar.

* **/kalman_filter_node/laserscan** ([std_msgs/Float64MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html)):*/kalman_filter_node/nearness* topic converted into a laser scan message.


#### Parameters
~ *covariance_process* (double, default: 100): process noise covariance diagonal values.

~ *covariance_optic_flow* (double, default: 5): covariance noise diagonal values of the radar measurement set.

~ *covariance_radar* (double,default: 0.001): covariance noise diagonal values of the optic flow measurement set.

~ *threshold* (double, default: 0.3): threshold to remove radar outliers.

~ *minimum_velocity* (double, default: 0.4): minimum velocity required to initialize the filter.

~ *lowess_smoothness* (double, default: 0.2): smoothness parameter *f* of the Lowess filter.
