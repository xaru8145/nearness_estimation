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
    //pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>("depth/laserscan", 10);

    // Import parameters
    nh_.param("/kf_nearness/num_ring_points", N_, 160);
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
    R_.resize(N_+Nrad_,N_+Nrad_);
    R_.topLeftCorner(N_, N_) = R_oflow_;
    R_.topRightCorner(N_,Nrad_).setZero();
    R_.bottomLeftCorner(Nrad_,N_).setZero();
    R_.bottomRightCorner(Nrad_,Nrad_) = R_rad_;

    // Initialize state
    state0_.setZero(N_);
    P0_.setIdentity(N_,N_);

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

}

void KalmanFilter::oflowCb(const std_msgs::Float32MultiArrayConstPtr &oflow_msg){

  flag_oflow_ = true;
  oflow_.resize(N_);
  gamma_vector_.resize(N_);
  for (int i = 0; i < N_; i++){
      oflow_(i) = oflow_msg->data[i];
      gamma_vector_(i) = ((float(i)/float(N_))*2*M_PI);
  }

}

void KalmanFilter::radarscanCb(const sensor_msgs::LaserScanConstPtr &radar_scan_msg){

  flag_radar_ = true;
  VectorXd y_rad(Nrad_), y_oflow(N_);
  for (int i = 0; i < Nrad_; i++){
    y_rad(i) = radar_scan_msg->ranges[i];
      if (!isinf(y_rad(i))){
        y_rad(i) = 1/y_rad(i);
      }
  }
  // Reverse to match the indexing optic flow measurements
  y_rad_ = y_rad.reverse();

  ros::Time current_timestamp = ros::Time::now();

  if (init_){
    state_update_ = state0_;
    P_update_ = P0_;
    init_ = false;
  }
  else {
  //if (flag_radar_ && flag_oflow_ && flag_imu_){
    //ros::Time current_timestamp = ros::Time::now();
  //}
  // Run filter when robot is moving and we receive all meas
      if (flag_radar_ && flag_oflow_ && flag_imu_ && flag_vel_){
        //ROS_INFO("All flags activated");
        // Measurement vector
        y_.resize(N_+Nrad_);
        y_oflow = oflow_.array() + r_;
        y_ << y_oflow, y_rad_; // Join measurements

        // Calculate derivates for the process model
        dt_ = (current_timestamp - last_timestamp_).toSec();
        VectorXd doflow = (oflow_ - last_oflow_);
        doflow /= dt_;
        double dr = (r_ - last_r_)/dt_;
        double du = (u_ - last_u_)/dt_;
        double dv = (v_ - last_v_)/dt_;

        // Create process and measurement models
        VectorXd h(N_);
        VectorXd a(N_);
        for (int i = 0; i < N_; i++){
          h(i) = u_*sin(gamma_vector_(i)) - v_*cos(gamma_vector_(i));
          a(i) = (doflow(i) + dr)/(oflow_(i) + r_) - (du*sin(gamma_vector_(i)) - dv*cos(gamma_vector_(i)))/h(i);
          // Set nan values of a to 0
          if(isnan(a(i))){
            a(i) = 0;
          }
        }
        MatrixXd H_oflow = h.asDiagonal();
        MatrixXd H_radar(Nrad_,N_);
        H_radar << MatrixXd::Zero(Nrad_,(N_-Nrad_)/2), MatrixXd::Identity(Nrad_,Nrad_), MatrixXd::Zero(Nrad_,(N_-Nrad_)/2);
        MatrixXd H(N_+Nrad_,N_);
        H.topLeftCorner(N_, N_) = H_oflow;
        H.bottomLeftCorner(Nrad_,N_) = H_radar;
        MatrixXd A = a.asDiagonal();
        MatrixXd F = MatrixXd::Identity(N_,N_) + A*dt_ + A*A*dt_*dt_/2 + A*A*A*dt_*dt_*dt_/6;

        // Predict
        VectorXd state_pred = F*last_state_;
        MatrixXd mat1 = F*last_P_*F.transpose();
        MatrixXd P_pred = mat1 + Q_;
        P_pred = 0.5*(P_pred + P_pred.transpose()); //enforce symmestry
        // Restriction of contact
        for (int i = 0; i < N_; i++){
          if(state_pred(i)>5){
            state_pred(i) = 5;
          }
          else if(state_pred(i)<-5){
            state_pred(i) = -5;
          }
        }
        // Void rows of H with non-valid radar measurements (=inf)
        for (int i = 0; i < Nrad_; i++){
          if(isinf(y_(N_+i))){
            H.row(N_+i).setZero();
          }
        }

        // Calculate Kalman gain
        MatrixXd mat2 = (H*P_pred*H.transpose() + R_);
        MatrixXd K = P_pred*H.transpose()*mat2.cwiseInverse();
        // Void columns of K with non-valid radar measurements (=inf)
        for (int i = 0; i < Nrad_; i++){
          if(isinf(y_(N_+i))){
            K.col(N_+i).setZero();
          }
        }

        //Update
        state_update_ = state_pred + K*(y_-H*state_pred);
        // No readable nearness by definition @ center points
        state_update_(0) = 0;
        state_update_(N_/2-1) = 0;
        P_update_ = (MatrixXd::Identity(N_,N_) - K*H)*P_pred;

        // Reset flags
        flag_radar_ = false;
        flag_oflow_ = false;
        flag_imu_ = false;
        flag_vel_ = false;

        std_msgs::Float32MultiArray mu_msg;
        for(int i = 0; i <N_; i++){
            mu_msg.data.push_back( state_update_(i) );
        }
        pub_mu_.publish(mu_msg);
      }
    }


last_timestamp_ = current_timestamp;
last_oflow_ = oflow_;
last_r_ = r_;
last_u_ = u_;
last_v_ = v_;
last_state_ = state_update_;
last_P_ = P_update_;

}
