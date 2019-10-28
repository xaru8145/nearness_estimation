#include <depth_estimation/depth_estimation.h>

DepthEstimation::DepthEstimation(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &private_node_handle)
        :nh_(node_handle),
        pnh_(private_node_handle) {
        this->init();
  }

void DepthEstimation::init() {

    init_ = true;

    // Set up subscribers and callbacks
    sub_state_ = nh_.subscribe("/vrpn2odom/odom", 1, &DepthEstimation::stateCb, this);
    sub_tang_flow_ = nh_.subscribe("tang_optic_flow", 1, &DepthEstimation::oflowCb, this);
    //Define lidar sub

    // Set up publishers
    pub_depth_ = nh_.advertise<std_msgs::Float32MultiArray>("estimated_depth", 10);

    // Import parameters
    nh_.param("/optic_flow_node/num_ring_points", num_ring_points_, 30);

} // End of init


void DepthEstimation::stateCb(const nav_msgs::OdometryConstPtr &state_msg){

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(state_msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(phi_, theta_, psi_);

  //Negative signs are to correct for frame mismatch between Vicon frame and NED
  theta_ = - theta_;
  psi_ = - psi_;

  u_ = state_msg->twist.twist.linear.x;
  v_ = state_msg->twist.twist.linear.y;
  r_ = state_msg->twist.twist.angular.z;

}

void DepthEstimation::oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg){

  ave_tang_flow_.resize(num_ring_points_);
  for(int i = 0; i < num_ring_points_; i++){
          ave_tang_flow_(i) = oflow_msg->data[i];
  }

}

void DepthEstimation::depthCb(){

  //Calculate gamma vector
  VectorXf V(num_ring_points_), mu_vector(num_ring_points_), Qr(num_ring_points_);

  gamma_vector_.resize(num_ring_points_);
  for(int i = 0; i < num_ring_points_; i++){
        gamma_vector_(i) = ((float(i)/float(num_ring_points_-1))*2*M_PI - M_PI);
        V(i) = ( u_*sin(gamma_vector_(i)) - v_*cos(gamma_vector_(i)) );
        Qr(i) = ave_tang_flow_(i) + r_;
  }

  //Calculate depth
  mu_vector = (V.transpose()*V).inverse()*V.transpose()*( Qr );
  depth_vector_.resize(num_ring_points_);
  depth_vector_ = mu_vector.inverse();

  std_msgs::Float32MultiArray depth_msg;
  for(int i = 0; i < num_ring_points_; i++){
          depth_msg.data.push_back( depth_vector_(i) );
  }

}
