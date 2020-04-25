#include <nearness_estimation/kalman_filter.h>
#include <CppLowess/Lowess.h>

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
    sub_tang_flow_ = nh_.subscribe("/optic_flow_node/tang_optic_flow/filtered", 1, &KalmanFilter::oflowCb, this);
    sub_odom_ = nh_.subscribe("/odometry/filtered", 1, &KalmanFilter::odomCb, this);
    //Define lidar sub

    // Set up publishers
    pub_mu_ = nh_.advertise<std_msgs::Float64MultiArray>("nearness", 10);
    pub_laser_ = nh_.advertise<sensor_msgs::LaserScan>("laserscan", 10);

    // Import parameters
    nh_.param("/kf_nearness_node/num_ring_points", N_,160);
    nh_.param("/kf_nearness_node/covariance_process", q_, 100.0);
    nh_.param("/kf_nearness_node/covariance_optic_flow", r_oflow_, 5.0);
    nh_.param("/kf_nearness_node/covariance_radar", r_rad_, 0.001);
    nh_.param("/kf_nearness_node/threshold", thresh_, 0.3);
    nh_.param("/kf_nearness_node/lowess_smoothness", f_smooth_, 0.2);
    nh_.param("/kf_nearness_node/minimum_velocity", min_vel_, 0.4);
    nh_.param("/pointcloud_to_laserscan/angle_max", a_max, 0.78);
    nh_.param("/pointcloud_to_laserscan/angle_min", a_min, -0.78);
    nh_.param("/pointcloud_to_laserscan/angle_increment", da, 0.03);
    Nrad_ = (a_max - a_min)/da;
    Ndy_ = (N_+Nrad_)/2;

    // Process noise covariance
    Q_.setIdentity(N_,N_);
    Q_ = Q_*q_;
    // Higher covariance @ front of the vehicle
    for (int i = N_/2-10; i < N_/2+10; i++){
      Q_(i,i) = Q_(i,i)*1000; // Test x5 and x10 of that
    }

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
    P0_ = Q_;

    //Define sizes
    state_.resize(N_);
    state_pred_.resize(N_);
    state_future_.resize(N_);
    f_.resize(N_);
    y_.resize(N_+Nrad_);
    P_update_.resize(N_,N_);
    P_pred_.resize(N_,N_);
    k_ = 1;
    }

void KalmanFilter::radarvelCb(const geometry_msgs::TwistWithCovarianceStampedConstPtr &radar_vel_msg){

  u_ = radar_vel_msg->twist.twist.linear.x;
  v_ = 0; // Non-holonomic constraints
  if (u_>min_vel_){
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

void KalmanFilter::odomCb(const nav_msgs::OdometryConstPtr &odom_msg){

  flag_odom_ = true;
  pos_x_ = odom_msg->pose.pose.position.x;
  pos_y_ = odom_msg->pose.pose.position.y;

}

void KalmanFilter::radarscanCb(const sensor_msgs::LaserScanConstPtr &radar_scan_msg){

  flag_radar_ = true;
  y_rad_.resize(Nrad_);
  for (int i = Nrad_; i >0; i--){
    y_rad_(Nrad_-i) = radar_scan_msg->ranges[i];
      if (!isinf(y_rad_(Nrad_-i))){
        y_rad_(Nrad_-i) = 1/y_rad_(Nrad_-i);
      }
  }

  ros::Time current_timestamp = ros::Time::now();
  if (init_){
    state_ = state0_;
    P_update_ = P0_;
    init_ = false;
  }
  else {
    if (flag_radar_ && flag_oflow_ && flag_imu_ && flag_vel_ && flag_odom_){
    // Run filter when robot is moving and all measureents are received
        VectorXd y_oflow(N_);
        VectorXd oflow2mu(N_);
        for(int i = 0; i <N_; i++){
            oflow2mu(i) = (oflow_(i)+r_)/(u_*sin(gamma_vector_(i))-v_*cos(gamma_vector_(i)));
            if (oflow2mu(i)<0 || i==N_/2 || i==N_/2+1 || i==N_/2+2){
              // Force measured oflow has positive nearness associated
              // Around 180deg, meas are not very reliable.
              oflow_(i) = -r_;
              oflow2mu(i) = (oflow_(i)+r_)/(u_*sin(gamma_vector_(i))-v_*cos(gamma_vector_(i)));
            }
            y_oflow(i) = oflow_(i) + r_;
        }

        y_.setZero();
        y_ << y_oflow, y_rad_; // Join measurements

        // Calculate derivates for the process model
        dt_ = (current_timestamp - last_timestamp_).toSec();
        VectorXd doflow(N_);
        double dr = (r_ - last_r_)/dt_;
        double du = (u_ - last_u_)/dt_;
        double dv = (v_ - last_v_)/dt_;

        // Create process and measurement models
        VectorXd h(N_);
        VectorXd a(N_);
        f_.setZero();
        for (int i = 0; i < N_; i++){
          doflow(i) = (oflow_(i) - last_oflow_(i))/dt_;
          h(i) = u_*sin(gamma_vector_(i)) - v_*cos(gamma_vector_(i));
          a(i) = (doflow(i) + dr)/(oflow_(i) + r_) - (du*sin(gamma_vector_(i)) - dv*cos(gamma_vector_(i)))/h(i);
          // Set nan values of a to 0
          if (isnan(a(i)) || isinf(a(i))){
            a(i) = 0;
          }
          else if (a(i)>50){
            a(i) = 50;
          }
          else if (a(i)<-50){
            a(i) = -50;
          }
          f_(i) = 1 + a(i)*dt_ + a(i)*a(i)*dt_*dt_/2 + a(i)*a(i)*a(i)*dt_*dt_*dt_/6;
        }

        MatrixXd H_oflow = h.asDiagonal();
        MatrixXd H_radar(Nrad_,N_);
        H_radar << MatrixXd::Zero(Nrad_,(N_-Nrad_)/2), MatrixXd::Identity(Nrad_,Nrad_), MatrixXd::Zero(Nrad_,(N_-Nrad_)/2);
        H_.resize(N_+Nrad_,N_);
        H_.topLeftCorner(N_, N_) = H_oflow;
        H_.bottomLeftCorner(Nrad_,N_) = H_radar;

        if (k_>1){
          removeOutliers();
        }

        predict();

        kalmanGain();

        update();

        filterData();

        publishLaser();

        // Reset flags
        flag_radar_ = false;
        flag_oflow_ = false;
        flag_imu_ = false;
        flag_vel_ = false;
        flag_odom_ = false;

        // Published filtered estimated nearness
        std_msgs::Float64MultiArray mu_msg;
        for(int i = 0; i <N_; i++){
            mu_msg.data.push_back( state_(i) );
        }
        pub_mu_.publish(mu_msg);
        k_ = k_+1;
    }
  }

last_timestamp_ = current_timestamp;
last_oflow_ = oflow_;
last_state_ = state_;
last_pos_x_ = pos_x_;
last_pos_y_ = pos_y_;
last_r_ = r_;
last_u_ = u_;
last_v_ = v_;
last_P_ = P_update_;

}

// Functions
void KalmanFilter::predict(){
    P_pred_.setZero();
    state_pred_.setZero();
    for (int i = 0; i < N_; i++){
      state_pred_(i) = f_(i)*last_state_(i);
      // Restriction of contact
      if(state_pred_(i)>5){
        state_pred_(i) = 5;
      }
      else if(state_pred_(i)<-5){
        state_pred_(i) = -5;
      }
      P_pred_(i,i) = f_(i)*last_P_(i,i)*f_(i) + Q_(i,i);
    }
    // Void rows of H with non-valid radar measurements (=inf)
    for (int i = 0; i < Nrad_; i++){
      if (isinf(y_(N_+i))){
        H_.row(N_+i).setZero();
      }
    }
}

void KalmanFilter::removeOutliers(){
    float odom_x = pos_x_ - last_pos_x_;
    float odom_y = pos_y_ - last_pos_y_;
    float gamma_approx, dx, dy;
    VectorXf diff(N_);
    VectorXi index(N_);
    state_future_.setZero();
    for (int i = 0; i < N_; i++){
      dx = 1/last_state_(i)*cos(gamma_vector_(i)) + odom_x;
      dy = 1/last_state_(i)*sin(gamma_vector_(i)) + odom_y;
      gamma_approx = atan2f(dy,dx);
      gamma_approx = (gamma_approx > 0 ? gamma_approx : (2*M_PI + gamma_approx)) ; // map atan to 0-2pi
      diff = abs(gamma_vector_.array() - gamma_approx);
      // Search for min diff
      index(i) = 0;
      for (int l = 1; l < N_; l++){
        if (diff(l)<diff(l-1)){
          index(i) = l;
        }
      }
      state_future_(index(i)) = 1/((1/last_state_(i)*sin(gamma_vector_(i)) + odom_y)/sin(gamma_approx));
    }
    // Fill non assigned indices within radar data & remove outliers
    for (int i = (N_-Nrad_)/2; i < (N_-Nrad_)/2+Nrad_; i++){
      if (state_future_(i)==0){
        if (state_future_(i-1)==0 && state_future_(i+1)!=0){
          state_future_(i) = state_future_(i+1);
        }
        else if (state_future_(i-1)==0 && state_future_(i+1)!=0){
          state_future_(i) = state_future_(i-1);
        }
        else{
          state_future_(i) = 0.5*state_future_(i-1) + 0.5*state_future_(i+1);
        }
      }
      if (!isinf(y_(Ndy_+i))){
        if (abs(state_future_(i) - y_(Ndy_+i))>thresh_){
            y_(Ndy_+i) = std::numeric_limits<double>::infinity();
        }
      }
    }
}

void KalmanFilter::kalmanGain(){
    // Calculate H*P*H'+R and P*H'
    MatrixXd pht;
    MatrixXd hphtR;
    pht.setZero(N_,N_+Nrad_);
    hphtR.setZero(N_+Nrad_,N_+Nrad_);
    for (int i = 0; i < N_+Nrad_; i++){
      if (i<N_){
        pht(i,i) = P_pred_(i,i)*H_(i,i);
        hphtR(i,i) = H_(i,i)*pht(i,i) + R_(i,i);
      }
      else{
        pht(i-Ndy_,i) = P_pred_(i-Ndy_,i-Ndy_)*H_(i,i-Ndy_);
        hphtR(i,i) = pht(i-Ndy_,i)*H_(i,i-Ndy_) + R_(i,i);
        hphtR(i-Ndy_,i) = pht(i-Ndy_,i)*H_(i-Ndy_,i-Ndy_);
        hphtR(i,i-Ndy_) = pht(i-Ndy_,i-Ndy_)*H_(i,i-Ndy_);
      }
    }

    // Calculate inverse(hphtR)
    double det;
    MatrixXd invhphtR;
    invhphtR.setZero(N_+Nrad_,N_+Nrad_);
    for (int i = 0; i < Ndy_; i++){
        if(i<(N_-Nrad_)/2){
          invhphtR(i,i) = 1/hphtR(i,i);
          invhphtR(i+Ndy_,i+Ndy_) = 1/hphtR(i+Ndy_,i+Ndy_);
        }
        else{
          det = 1/(hphtR(i,i)*hphtR(i+Ndy_,i+Ndy_) - hphtR(i+Ndy_,i)*hphtR(i,i+Ndy_));
          //ROS_INFO("11: %f, 22: %f, 12: %f, det: %f", hphtR(i,i),hphtR(i+Ndy_,i+Ndy_),hphtR(i+Ndy_,i),det);
          invhphtR(i,i) = det*hphtR(i+Ndy_,i+Ndy_);
          invhphtR(i+Ndy_,i) = -det*hphtR(i+Ndy_,i);
          invhphtR(i,i+Ndy_) =  invhphtR(i+Ndy_,i);
          invhphtR(i+Ndy_,i+Ndy_) = det*hphtR(i,i);
        }
    }

    // K = P*H'*inverse(H*P*H'+R)
    K_.resize(N_,N_+Nrad_);
    K_.setZero();
    for (int i = 0; i <N_+Nrad_; i++){
        if(i<(N_-Nrad_)/2){
          K_(i,i) = pht(i,i)*invhphtR(i,i);
        }
        else if(i>=(N_-Nrad_)/2 && i<(N_-Nrad_)/2+Nrad_){
          K_(i,i) = pht(i,i)*invhphtR(i,i) + pht(i,i+Ndy_)*invhphtR(i+Ndy_,i);
        }
        else if(i>=(N_-Nrad_)/2+Nrad_ && i<N_){
          K_(i,i) = pht(i,i)*invhphtR(i,i);
        }
        else {
          K_(i-Ndy_,i) = pht(i-Ndy_,i-Ndy_)*invhphtR(i-Ndy_,i) + pht(i-Ndy_,i)*invhphtR(i,i);
        }
    }

    // Void columns of K with non-valid radar measurements (=inf)
    for (int i = 0; i < Nrad_; i++){
      if(isinf(y_(N_+i))){
        K_.col(N_+i).setZero();
        y_(N_+i) = 1.5;
      }
    }
}

void KalmanFilter::update()
{
    // Calculate K*H
    KH_.setZero(N_,N_);
    for (int i = 0; i < N_; i++){
        KH_(i,i) = H_(i,i)*K_(i,i);
        if (i>=(N_-Nrad_)/2 && i<(N_-Nrad_)/2+Nrad_){
          KH_(i,i) =  H_(i,i)*K_(i,i) + K_(i,i+Ndy_)*H_(i+Ndy_,i);
        }
    }
    state_.setZero();
    P_update_. setZero();
    for (int i = 0; i < N_; i++){
      state_(i) = state_pred_(i) + K_(i,i)*(y_(i) - H_(i,i)*state_pred_(i));
      if (i>=(N_-Nrad_)/2 && i<(N_-Nrad_)/2+Nrad_){
        state_(i) = state_pred_(i) + K_(i,i)*(y_(i) - H_(i,i)*state_pred_(i)) + K_(i,i+Ndy_)*(y_(i+Ndy_) - H_(i+Ndy_,i)*state_pred_(i));
      }
      P_update_(i,i) = P_pred_(i,i) - KH_(i,i)*P_pred_(i,i);
    }
    // No readable nearness by definition @ center points
    state_(0) = 0;
    state_(N_/2) = 0;
}


void KalmanFilter::filterData()
{
  // Apply LOESS filter
  // https://github.com/hroest/CppLowess
  CppLowess::TemplatedLowess<std::vector<double>, double> dlowess;
  vector<double> v_xval;
  vector<double> v_yval;
   for (size_t i = 0; i < Nrad_; i++)
   {
     v_xval.push_back(gamma_vector_(i + (N_-Nrad_)/2));
     v_yval.push_back(state_(i + (N_-Nrad_)/2));
   }

  vector<double> filtered_y(Nrad_), tmp1(Nrad_), tmp2(Nrad_);
  dlowess.lowess(v_xval, v_yval, f_smooth_, 1, 0.0, filtered_y, tmp1, tmp2); //f,nsteps,delta
  for (size_t i = 0; i < Nrad_; i++){
    state_(i + (N_-Nrad_)/2) = filtered_y[i];
  }
}


void KalmanFilter::publishLaser(){
  // Publish estimated nearness into laser scan
  ros::Time scan_time = ros::Time::now();

  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = scan_time;
  laser_msg.header.frame_id = "oflow_laser_frame";
  laser_msg.angle_min = -M_PI;
  laser_msg.angle_max = M_PI;
  laser_msg.angle_increment = 2*M_PI / N_;
  laser_msg.time_increment =  0.04 / N_;
  laser_msg.range_min = 0.2;
  laser_msg.range_max = 10.0;
  laser_msg.ranges.resize(N_);
  laser_msg.intensities.resize(N_);
  for(unsigned int i = 0; i < N_; ++i){
     laser_msg.ranges[i] = 1/state_(i);
     if (state_(i)<0){
       laser_msg.ranges[i] = std::numeric_limits<double>::infinity();
     }
     // Void bad reading areas in the front
     if (i>=N_/2-10 && i < N_/2+10){
       laser_msg.ranges[i] = std::numeric_limits<double>::infinity();
     }
     // Void bad reading areas in the back
     else if (i<10 && i >= N_-10){
       laser_msg.ranges[i] = std::numeric_limits<double>::infinity();
     }
   }
  pub_laser_.publish(laser_msg);
}
