#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // TODO: tune
  std_a_ = 1; // 30; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  // TODO: tune
  std_yawdd_ = M_PI / 30; // 30; 
  
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
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // state dimention
  n_x_ = 5;

  // augmented dimension
  n_aug_ = 7;

  // sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // state vector
  x_ = VectorXd(n_x_);

  // state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // augemented state vector
  x_aug_ = VectorXd(n_aug_);

  // augmented covariance matrix
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  // augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  /**
   * Initialization
   */
  if (!is_initialized_) {
    time_us_ = meas_package.timestamp_;

    const VectorXd &z = meas_package.raw_measurements_;

    // state vector
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      x_ << z(0) * cos(z(1)), z(0) * sin(z(1)), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << z(0), z(1), 0, 0, 0;
    }

    // state covariance matrix
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1.0e+6;
  time_us_ = meas_package.timestamp_;

  /**
   * Prediction
   */

  Prediction(dt);

  /**
   * Update
   */

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }
}

void UKF::Prediction(const double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //std_a_;
  //std_yawdd_;
  //x_ = F_ * x_;
  //P_ = F_ * P_ * F_.transpose() + Q_;

  // augemented state vector
  x_aug_.head(n_x_) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  // augmented covariance matrix
  P_aug_.fill(0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(5, 5) = std_a_ * std_a_;
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

  // generate augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  MatrixXd A = P_aug_.llt().matrixL();
  for (unsigned int i = 0; i < n_aug_; ++i) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * A.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  // predict sigma points at next time step
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {

      double p_x = Xsig_aug_(0, i);
      double p_y = Xsig_aug_(1, i);
      double v = Xsig_aug_(2, i);
      double yaw = Xsig_aug_(3, i);
      double yawd = Xsig_aug_(4, i);
      double nu_a = Xsig_aug_(5, i);
      double nu_yawdd = Xsig_aug_(6, i);

      double px_p, py_p;
      if (fabs(yawd) > div_eps) {
          px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
          py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
      }
      else {
          px_p = p_x + v * delta_t * cos(yaw);
          py_p = p_y + v * delta_t * sin(yaw);
      }
      double v_p = v;
      double yaw_p = yaw + yawd * delta_t;
      double yawd_p = yawd;

      // add process noise
      px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
      py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
      v_p += nu_a * delta_t;
      yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
      yawd_p += nu_yawdd * delta_t;

      // store predicted sigma point
      Xsig_pred_(0, i) = px_p;
      Xsig_pred_(1, i) = py_p;
      Xsig_pred_(2, i) = v_p;
      Xsig_pred_(3, i) = yaw_p;
      Xsig_pred_(4, i) = yawd_p;
  }

  // sigma points weights  
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // predicted state mean
  x_.fill(0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {
      x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  //x_ = Xsig_pred_ * weights_.transpose();
  //x_ = Xsig_pred_ * weights_;

  // predicted state covariance matrix
  P_.fill(0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      
      // angle normalization
      //NormalizePhi(&x_diff);
      while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

      P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::NormalizePhi(VectorXd *x) {

    // make sure phi is in the range (-pi, pi)
    double phi = (*x)(3);
    phi += (phi > 0) ? M_PI : -M_PI;
    double pi2 = 2. * M_PI;
    int div = phi / pi2;
    (*x)(1) -= div * pi2;
}

void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  MatrixXd H_lidar_(2, 5);
  H_lidar_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  MatrixXd R_lidar_(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;            

  VectorXd y = meas_package.raw_measurements_ - H_lidar_ * x_;
  MatrixXd S = H_lidar_ * P_ * H_lidar_.transpose() + R_lidar_;
  MatrixXd K = P_ * H_lidar_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_lidar_) * P_;
}

void UKF::UpdateRadar(const MeasurementPackage &meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  MatrixXd R_radar_(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  // ...
}