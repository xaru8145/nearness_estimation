#include <nearness_estimation/kalman_filter.h>

KalmanFilter::KalmanFilter(const ros::NodeHandle &node_handle,
                           const ros::NodeHandle &private_node_handle)
        :nh_(node_handle),
        pnh_(private_node_handle) {
        this->init();
  }

void KalmanFilter::init() {

    double a_max, a_min, da;

    init_ = true;
    flag_imu_ = false;
    flag_oflow_ = false;
    flag_radar_ = false;
    flag_vel_ = false;

    // Set up subscribers and callbacks
    sub_radar_vel_ = nh_.subscribe("/mmWaveDataHdl/velocity", 1, &KalmanFilter::radarvelCb, this);
    sub_radar_scan_ = nh_.subscribe("/radar_scan", 1, &KalmanFilter::radarscanCb, this);
    sub_imu_ = nh_.subscribe("/imu/data_added_cov", 1, &KalmanFilter::imuCb, this);
    sub_tang_flow_ = nh_.subscribe("/optic_flow_node/tang_optic_flow", 1, &KalmanFilter::oflowCb, this);
    //Define lidar sub

    // Set up publishers
    pub_mu_ = nh_.advertise<std_msgs::Float32MultiArray>("nearness/kf", 10);
    pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>("depth/laserscan", 10);

    // Import parameters
    nh_.param("/kf_nearness/num_ring_points", N_, 80);
    nh_.param("/kf_nearness/covariance_process", q_, 100.0);
    nh_.param("/kf_nearness/covariance_optic_flow", r_oflow_, 5.0);
    nh_.param("/kf_nearness/covariance_radar", r_rad_, 0.001);
    nh_.param("/pointcloud_to_laserscan/angle_max", a_max, 0.78);
    nh_.param("/pointcloud_to_laserscan/angle_min", a_min, -0.78);
    nh_.param("/pointcloud_to_laserscan/angle_increment", da, 0.03);
    Nrad_ = 40; //for comfort before finishing the code(a_max - a_min)/da;

    // Process noise covariance
    Q_.setIdentity(N_,N_);
    Q_ = Q_*q_;

    // Measurement noise covariance
    R_oflow_.setIdentity(N_,N_);
    R_oflow_ = R_oflow_*r_oflow_;
    R_rad_.setIdentity(Nrad_,Nrad_);
    R_rad_ = R_rad_*r_rad_;
    //R_rad_ =

    }

void KalmanFilter::radarvelCb(const geometry_msgs::TwistWithCovarianceStampedConstPtr &radar_vel_msg){

  u_ = radar_vel_msg->twist.twist.linear.x;
  // Set lat velocity to 0 for now
  v_ = 0;
  if (u_>0.4){
    flag_vel_ = true; //robot is moving
  }

}

void KalmanFilter::imuCb(const sensor_msgs::ImuConstPtr &imu_msg){

  flag_imu_ = true;
  // Angular rate of filtered IMU data on z axis
  r_ = imu_msg->angular_velocity.z;
  imu_time_ = imu_msg-> header.stamp;

}

void KalmanFilter::oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg){

  flag_oflow_ = true;
  ave_tang_flow_.resize(N_);
  for(int i = 0; i < N_; i++){
          ave_tang_flow_(i) = oflow_msg->data[i];
  }

}

void KalmanFilter::radarscanCb(const sensor_msgs::LaserScanConstPtr &radar_scan_msg){

  flag_radar_ = true;
  //y_rad_ = radar_scan_msg->ranges;

}
