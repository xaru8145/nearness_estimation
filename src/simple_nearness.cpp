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
    sub_imu_ = nh_.subscribe("/imu/data", 1, &NearnessEstimation::imuCb, this);
    sub_tang_flow_ = nh_.subscribe("/optic_flow_node/tang_optic_flow", 1, &NearnessEstimation::oflowCb, this);
    //Define lidar sub

    // Set up publishers
    pub_mu_ = nh_.advertise<std_msgs::Float32MultiArray>("estimated_nearness", 10);

    // Import parameters
    nh_.param("/simple_nearness/num_ring_points", num_ring_points_, 80);

} // End of init


void NearnessEstimation::radarCb(const geometry_msgs::TwistWithCovarianceStampedConstPtr &radar_msg){

  u_ = radar_msg->twist.twist.linear.x;
  // Set lat velocity to 0 for now
  v_ = 0;

}

void NearnessEstimation::imuCb(const sensor_msgs::ImuConstPtr &imu_msg){

  // Angular rate of filtered IMU data on z axis
  r_ = imu_msg->angular_velocity.z;

}

void NearnessEstimation::oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg){

  ave_tang_flow_.resize(num_ring_points_);
  for(int i = 0; i < num_ring_points_; i++){
          ave_tang_flow_(i) = oflow_msg->data[i];
  }

  //ROS_INFO_THROTTLE(1,"r  %f", r_);

  //Calculate gamma vector and nearness
  gamma_vector_.resize(num_ring_points_);
  mu_vector_.resize(num_ring_points_);

  for(int i = 0; i < num_ring_points_; i++){
        gamma_vector_(i) = ((float(i)/float(num_ring_points_-1))*2*M_PI);// - M_PI);
        //cout << gamma_vector_(i);
        //ROS_INFO_THROTTLE("gamma  %f", gamma_vector_(i));
	      //Ignore lateral velocity for now
	      mu_vector_(i) = ( ave_tang_flow_(i) + r_ )/(u_*sin(gamma_vector_(i)));
  }

  std_msgs::Float32MultiArray mu_msg;
  for(int i = 0; i < num_ring_points_; i++){
          mu_msg.data.push_back( mu_vector_(i) );
  }

  pub_mu_.publish(mu_msg);

}
