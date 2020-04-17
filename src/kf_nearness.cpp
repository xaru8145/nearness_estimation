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
    nh_.param("/kf_nearness/num_ring_points", N_, 120);
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
    identity_.setIdentity(N_,N_);

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
  // Run filter when robot is moving and we receive all meas
      if (flag_radar_ && flag_oflow_ && flag_imu_ && flag_vel_){
        //ROS_INFO("All flags activated");
        // Measurement vector
        y_.resize(N_+Nrad_);
        VectorXd y_oflow(N_);
        y_oflow = oflow_.array() + r_;
        y_ << y_oflow, y_rad_; // Join measurements

        // Calculate derivates for the process model
        dt_ = (current_timestamp - last_timestamp_).toSec();
        VectorXd doflow = (oflow_ - last_oflow_)/dt_;
        double dr = (r_ - last_r_)/dt_;
        double du = (u_ - last_u_)/dt_;
        double dv = (v_ - last_v_)/dt_;

        ros::Time time1 = ros::Time::now();

        // Create process and measurement models
        VectorXd h(N_);
        VectorXd a(N_);
        VectorXd f(N_);
        for (int i = 0; i < N_; i++){
          h(i) = u_*sin(gamma_vector_(i)) - v_*cos(gamma_vector_(i));
          a(i) = (doflow(i) + dr)/(oflow_(i) + r_) - (du*sin(gamma_vector_(i)) - dv*cos(gamma_vector_(i)))/h(i);
          // Set nan values of a to 0
          if(isnan(a(i))){
            a(i) = 0;
          }
          f(i) = 1 + a(i)*dt_ + a(i)*a(i)*dt_*dt_/2 + a(i)*a(i)*a(i)*dt_*dt_*dt_/6;
        }
        MatrixXd H_oflow = h.asDiagonal();
        MatrixXd H_radar(Nrad_,N_);
        H_radar << MatrixXd::Zero(Nrad_,(N_-Nrad_)/2), MatrixXd::Identity(Nrad_,Nrad_), MatrixXd::Zero(Nrad_,(N_-Nrad_)/2);
        H_.resize(N_+Nrad_,N_);
        H_.topLeftCorner(N_, N_) = H_oflow;
        H_.bottomLeftCorner(Nrad_,N_) = H_radar;
        F_.resize(N_,N_);
        F_ = f.asDiagonal();

        ros::Time time2 = ros::Time::now();

        predict();

        ros::Time time3 = ros::Time::now();

        // Calculate Kalman gain
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
            pht(i-(N_+Nrad_)/2,i) = P_pred_(i-(N_+Nrad_)/2,i-(N_+Nrad_)/2);
            hphtR(i,i) = pht(i-(N_+Nrad_)/2,i) + R_(i,i);
            hphtR(i-(N_+Nrad_)/2,i) = pht(i-(N_+Nrad_)/2,i)*H_(i-(N_+Nrad_)/2,i-(N_+Nrad_)/2);
            hphtR(i,i-(N_+Nrad_)/2) = pht(i-(N_+Nrad_)/2,i-(N_+Nrad_)/2);
          }
        }

        K_.resize(N_,N_+Nrad_);
        K_.noalias() =  pht*hphtR.inverse();
        // Void columns of K with non-valid radar measurements (=inf)
        for (int i = 0; i < Nrad_; i++){
          if(isinf(y_(N_+i))){
            K_.col(N_+i).setZero();
          }
        }

        ros::Time time4 = ros::Time::now();

        KH_.setZero(N_,N_);
        for (int i = 0; i < N_; i++){
            KH_(i,i) = H_(i,i)*K_(i,i);
            if (i>=(N_-Nrad_)/2 && i<(N_-Nrad_)/2+Nrad_){
              KH_(i,i) = KH_(i,i) + K_(i,i+(N_+Nrad_)/2)*H_(i+(N_+Nrad_)/2,i);
            }
        }

        update();

        ros::Time time5 = ros::Time::now();

        // Reset flags
        flag_radar_ = false;
        flag_oflow_ = false;
        flag_imu_ = false;
        flag_vel_ = false;
        double dt1 = (time2-time1).toSec();
        double dt2 = (time3-time2).toSec();
        double dt3 = (time4-time3).toSec();
        double dt4 = (time5-time4).toSec();
        ROS_INFO("models: %f, predict: %f, kalman: %f, update: %f", dt1,dt2,dt3,dt4);

        std_msgs::Float32MultiArray mu_msg;
        for(int i = 0; i <N_; i++){
            mu_msg.data.push_back( state_(i) );
        }
        pub_mu_.publish(mu_msg);
      }
    }


last_timestamp_ = current_timestamp;
last_oflow_ = oflow_;
last_r_ = r_;
last_u_ = u_;
last_v_ = v_;
last_P_ = P_update_;

}

void KalmanFilter::predict(){
    state_.noalias() = F_*state_;
    VectorXd FP = F_.diagonal().cwiseProduct(last_P_.diagonal());
    VectorXd FPFt = F_.diagonal().cwiseProduct(FP);
    P_pred_.resize(N_,N_);
    P_pred_ = FPFt.asDiagonal();
    P_pred_.noalias() += Q_;
    // Restriction of contact
    for (int i = 0; i < N_; i++){
      if(state_(i)>5){
        state_(i) = 5;
      }
      else if(state_(i)<-5){
        state_(i) = -5;
      }
    }
    // Void rows of H with non-valid radar measurements (=inf)
    for (int i = 0; i < Nrad_; i++){
      if(isinf(y_(N_+i))){
        H_.row(N_+i).setZero();
      }
    }
}


void KalmanFilter::update()
{
    VectorXd KHx;
    KHx.noalias() = KH_*state_;
    state_.noalias() += K_*y_-KHx;
    // No readable nearness by definition @ center points
    state_(0) = 0;
    state_(N_/2-1) = 0;
    for (int i = 0; i < Nrad_; i++){
          P_update_(i,i) = P_pred_(i,i) + KH_(i,i)*P_pred_(i,i);
    }
}
