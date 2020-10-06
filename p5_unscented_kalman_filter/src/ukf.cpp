#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2 
  // maximum acceleration/deceleration ~= 5.0 m/s^2 
  // Ref: Bokare, P. S., & Maurya, A. K. (2017). Acceleration-deceleration behaviour of various vehicle types. 
  // Transportation research procedia, 25, 4733-4749.
  std_a_ = 5.0;//0.5*5.0; // 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // maximum yaw rate = 30 deg/s
  // maximum yawdd = 60 deg/s assuming 0.5 second to reach the maximum rate
  std_yawdd_ = 0.5*60.0*M_PI/180.0; // 30; 
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  
  // set number of states
  n_x_ = 5;

  // Lambda parameter
  lambda_ = 3 - n_x_;
  
  // Augmented state size
  n_aug_ = n_x_ + 2;
  
  // Initialize predicted sigma points matrix
  auto sigma_size = 2 * n_aug_ + 1;
  Xsig_pred_ = MatrixXd::Zero(n_x_, sigma_size);  
  
  // Set weights 
  weights_ = VectorXd(sigma_size);
  weights_.setConstant(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // Init lidar measurement noise covariance matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0.0,
              0.0, std_laspy_*std_laspy_;

  // Init radar measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0.0, 0.0,
              0.0, std_radphi_*std_radphi_, 0.0,
              0.0, 0.0, std_radrd_*std_radrd_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    // Initialize x_ = [px, py, v, yaw, yaw_dot]
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0, 0.0, 0.0;
    } 
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      x_ << rho * cos(phi), rho * sin(phi), 0.0, 0.0, 0.0;
    }
    else
    {
      x_.setZero();
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // dt in second
  double dt = static_cast<double>((meas_package.timestamp_ - time_us_) * 1e-6);

  // Prediction
  Prediction(dt);

  // Update
  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  } 
  else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    //std::cout<<">>>> Sensor type is invalid!"<< std::endl;
    return;
  }
 
  // Update time if sensor type is valid and in use
  time_us_ = meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //// 1. Generate Sigma Points
  // State vector: x = [px, py, v, yaw, yaw_dot]
  // Augmented state vector: x_aug = [x; nu_a; nu_yaw_ddot]
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0.0; // acceleration
  x_aug(n_x_ + 1) = 0.0; // yaw_ddot

  // Augmented process covariance: P_aug = [P, 0; 0, Q]
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;
  
  // Augmented sigma point matrix
  MatrixXd L = P_aug.llt().matrixL();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for(auto i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  
  //// 2. Predict Sigma Points
  double px, py, v, yaw, yaw_dot, nu_a, nu_yaw_ddot;
  for(auto i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    px = Xsig_aug(0, i);
    py = Xsig_aug(1, i);
    v = Xsig_aug(2, i);
    yaw = Xsig_aug(3, i);
    yaw_dot = Xsig_aug(4, i);
    nu_a = Xsig_aug(5, i);
    nu_yaw_ddot = Xsig_aug(6, i);

    // One-step ahead prediction
    if(fabs(yaw_dot) <= 0.001)
    {
      // straight line exception
      px += (v * cos(yaw) * delta_t) + (0.5 * delta_t * delta_t * cos(yaw) * nu_a);
      py += (v * sin(yaw) * delta_t) + (0.5 * delta_t * delta_t * sin(yaw) * nu_a);
      v += (delta_t * nu_a);
      yaw += (yaw_dot * delta_t) + (0.5 * delta_t * delta_t * nu_yaw_ddot);
      yaw_dot += (delta_t * nu_yaw_ddot);
    }
    else
    {
      px += (v / yaw_dot) * (sin(yaw + yaw_dot * delta_t) - sin(yaw)) + (0.5 * delta_t * delta_t * cos(yaw) * nu_a);
      py += (v / yaw_dot) * (-cos(yaw + yaw_dot * delta_t) + cos(yaw)) + (0.5 * delta_t * delta_t * sin(yaw) * nu_a);
      v += (delta_t * nu_a);
      yaw += (yaw_dot * delta_t) + (0.5 * delta_t * delta_t * nu_yaw_ddot);
      yaw_dot += (delta_t * nu_yaw_ddot);
    }

    // Update sigma point prediction matrix
    Xsig_pred_(0, i) = px;
    Xsig_pred_(1, i) = py;
    Xsig_pred_(2, i) = v;
    Xsig_pred_(3, i) = yaw;
    Xsig_pred_(4, i) = yaw_dot;
  }

  //// 3. Predict Mean & Covariance
  // Predicted state mean
  x_.setZero();
  for(auto i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted state covariance matrix
  P_.setZero();
  for(auto i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization to constraint it to (-pi, pi)
    x_diff(3) = std::remainder(x_diff(3), 2.0*M_PI);
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);

  // Measurement mean
  VectorXd z_pred = VectorXd::Zero(n_z);
  for(auto i=0; i < 2 * n_aug_ + 1; ++i)
  {
      z_pred += weights_(i) * Zsig.col(i);
  }

  // Measurement covariance
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for(auto i = 0; i < 2* n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    z_diff(1) = std::remainder(z_diff(1), 2.0*M_PI);
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_lidar_;

  // Calculate Tc 
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for(auto i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = std::remainder(z_diff(1), 2.0*M_PI);
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = std::remainder(x_diff(3), 2.0*M_PI);
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Update ukf gain, state mean and covariance
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  z_diff(1) = std::remainder(z_diff(1), 2.0*M_PI);
  x_ += K * z_diff;
  P_ += -K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  
  int n_z = 3;
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);
  double px, py, v, yaw, vx, vy;
  for(auto i=0; i < 2* n_aug_ +1; ++i)
  {
    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    yaw = Xsig_pred_(3, i);
    vx = v * cos(yaw);
    vy = v * sin(yaw);
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = ((px * vx) + (py * vy)) / Zsig(0, i);
  }

  // Measurement mean
  VectorXd z_pred = VectorXd::Zero(n_z);
  for(auto i=0; i < 2 * n_aug_ + 1; ++i)
  {
      z_pred += weights_(i) * Zsig.col(i);
  }

  // Measurement covariance
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for(auto i = 0; i < 2* n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    z_diff(1) = std::remainder(z_diff(1), 2.0*M_PI);
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_radar_;

  // Calculate Tc
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for(auto i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = std::remainder(z_diff(1), 2.0*M_PI);
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = std::remainder(x_diff(3), 2.0*M_PI);
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Update ukf gain, state mean and covariance
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  z_diff(1) = std::remainder(z_diff(1), 2.0*M_PI);
  x_ += K * z_diff;
  P_ += -K * S * K.transpose();
}
