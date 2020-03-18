#include <nearness_estimation/nearness_estimation.h>

NearnessEstimation::NearnessEstimation(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &private_node_handle)
        :nh_(node_handle),
        pnh_(private_node_handle) {
        this->init();
  }

void NearnessEstimation::init() {

    init_ = true;

    // Set up subscribers and callbacks
    sub_state_ = nh_.subscribe("/mmWaveDataHdl/velocity", 1, &NearnessEstimation::radarCb, this);
    sub_imu_ = nh_.subscribe("/imu/data_added_cov", 1, &NearnessEstimation::imuCb, this);
    sub_tang_flow_ = nh_.subscribe("/optic_flow_node/tang_optic_flow", 1, &NearnessEstimation::oflowCb, this);
    //Define lidar sub

    // Set up publishers
    pub_mu_ = nh_.advertise<std_msgs::Float32MultiArray>("estimated_nearness", 10);
    pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>("depth/laserscan", 10);

    // Import parameters
    nh_.param("/simple_nearness/num_ring_points", num_ring_points_, 80);
    nh_.param("/simple_nearness/publish_laser", publish_laser_, false);

} // End of init


void NearnessEstimation::radarCb(const geometry_msgs::TwistWithCovarianceStampedConstPtr &radar_msg){

  u_ = radar_msg->twist.twist.linear.x;
  // Set lat velocity to 0 for now
  v_ = 0;

}

void NearnessEstimation::imuCb(const sensor_msgs::ImuConstPtr &imu_msg){

  // Angular rate of filtered IMU data on z axis
  r_ = imu_msg->angular_velocity.z;
  imu_time_ = imu_msg-> header.stamp;

}

void NearnessEstimation::oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg){
  // Time stamp for laser msg
  ros::Time scan_time = ros::Time::now();

  ave_tang_flow_.resize(num_ring_points_);
  for(int i = 0; i < num_ring_points_; i++){
          ave_tang_flow_(i) = oflow_msg->data[i];
  }

  //ROS_INFO_THROTTLE(1,"r  %f", r_);

  //Calculate gamma vector and nearness
  gamma_vector_.resize(num_ring_points_);
  mu_vector_.resize(num_ring_points_);
        //cout << "Gamma: "  ;
  for(int i = 0; i < num_ring_points_; i++){
        gamma_vector_(i) = ((float(i)/float(num_ring_points_))*2*M_PI);// - M_PI);
        //cout << gamma_vector_(i);
        //cout << ", ";
        //ROS_INFO_THROTTLE("gamma  %f", gamma_vector_(i));
	      //Ignore lateral velocity for now
	      mu_vector_(i) = ( ave_tang_flow_(i) + r_ )/(u_*sin(gamma_vector_(i)));
  }
          //cout << " :End"  ;

  std_msgs::Float32MultiArray mu_msg;
  for(int i = 0; i < num_ring_points_; i++){
          mu_msg.data.push_back( mu_vector_(i) );
  }

  pub_mu_.publish(mu_msg);

  if(publish_laser_){
        sensor_msgs::LaserScan laser_msg;
        laser_msg.header.stamp = imu_time_;//scan_time;
        laser_msg.header.frame_id = "oflow_laser_frame";
        laser_msg.angle_min = -M_PI;
        laser_msg.angle_max = M_PI;
        laser_msg.angle_increment = 2*M_PI / num_ring_points_;
        laser_msg.time_increment = (1 / 25) / (num_ring_points_);
        laser_msg.range_min = 0.2;
        laser_msg.range_max = 10.0;
        laser_msg.ranges.resize(num_ring_points_);
        laser_msg.intensities.resize(num_ring_points_);
        for(unsigned int i = 0; i < num_ring_points_; ++i){
           laser_msg.ranges[i] = 1/mu_vector_(i);
           laser_msg.intensities[i] = 47.0;
         }
        pub_laser_.publish(laser_msg);
  }

}
