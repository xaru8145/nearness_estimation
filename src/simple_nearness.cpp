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
    sub_state_ = nh_.subscribe("/vrpn_velocity/oflow_xaru8145_frame/filtered", 1, &NearnessEstimation::stateCb, this);
    sub_tang_flow_ = nh_.subscribe("/optic_flow_node/tang_optic_flow", 1, &NearnessEstimation::oflowCb, this);
    //Define lidar sub

    // Set up publishers
    pub_mu_ = nh_.advertise<std_msgs::Float32MultiArray>("estimated_nearness", 10);

    // Import parameters
    nh_.param("/simple_nearness/num_ring_points", num_ring_points_, 80);

} // End of init


void NearnessEstimation::stateCb(const geometry_msgs::TwistStampedConstPtr &state_msg){


  // Convert Quaternion to RPY
  //tf::Quaternion tf_quat;
  //tf::quaternionMsgToTF(state_msg->pose.pose.orientation, tf_quat);
  //tf::Matrix3x3(tf_quat).getRPY(phi_, theta_, psi_);

  //Negative signs are to correct for frame mismatch between Vicon frame and NED
  //theta_ = - theta_;
  //psi_ = - psi_;

  u_ = state_msg->twist.linear.x;
  v_ = state_msg->twist.linear.y;
  r_ = state_msg->twist.angular.z;
  u_ = - u_;
  //ROS_INFO_THROTTLE(1,"u  %f", u_);
  //ROS_INFO_THROTTLE(1,"v  %f", v_);
  //ROS_INFO_THROTTLE(1,"r  %f", r_);
}

void NearnessEstimation::oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg){

  ave_tang_flow_.resize(num_ring_points_);
  for(int i = 0; i < num_ring_points_; i++){
          ave_tang_flow_(i) = oflow_msg->data[i];
  }

  //Calculate gamma vector and nearness
  gamma_vector_.resize(num_ring_points_);
  mu_vector_.resize(num_ring_points_);

  u_ = 1; //hardcoded velocity to check working depth
  for(int i = 0; i < num_ring_points_; i++){
        gamma_vector_(i) = ((float(i)/float(num_ring_points_-1))*2*M_PI - M_PI);
	//Ignore lateral velocity for now
	mu_vector_(i) = ( ave_tang_flow_(i) + r_ )/(u_*sin(gamma_vector_(i)));
  }

  std_msgs::Float32MultiArray mu_msg;
  for(int i = 0; i < num_ring_points_; i++){
          mu_msg.data.push_back( mu_vector_(i) );
  }
 
  pub_mu_.publish(mu_msg);

}
