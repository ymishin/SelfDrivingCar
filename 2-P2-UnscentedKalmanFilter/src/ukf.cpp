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
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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

  // weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {

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
    P_ = MatrixXd::Identity(n_x_, n_x_);

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
  
  // augemented state vector
  VectorXd x_aug(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // augmented covariance matrix
  MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // generate augmented sigma points
  MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  MatrixXd A = P_aug.llt().matrixL();
  for (unsigned int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  // predict sigma points at next time step
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {

      double p_x = Xsig_aug(0, i);
      double p_y = Xsig_aug(1, i);
      double v = Xsig_aug(2, i);
      double yaw = Xsig_aug(3, i);
      double yawd = Xsig_aug(4, i);
      double nu_a = Xsig_aug(5, i);
      double nu_yawdd = Xsig_aug(6, i);

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
  x_ = Xsig_pred_ * weights_;

  // predicted state covariance matrix
  P_.fill(0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_; // state difference
    NormalizePhi(&x_diff(3)); // angle normalization
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateRadar(const MeasurementPackage &meas_package) {

  int n_z = 3;

  // sigma points in measurement space
  MatrixXd Zsig(n_z, 2 * n_aug_ + 1);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {

      double p_x = Xsig_pred_(0, i);
      double p_y = Xsig_pred_(1, i);
      double v = Xsig_pred_(2, i);
      double yaw = Xsig_pred_(3, i);

      double v1 = cos(yaw) * v;
      double v2 = sin(yaw) * v;

      Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
      Zsig(1, i) = atan2(p_y, p_x);
      Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);
  }

  // mean predicted measurement
  VectorXd z_pred = Zsig  * weights_;

  // measurement covariance matrix
  MatrixXd S(n_z, n_z);
  S.fill(0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred; // residual
    NormalizePhi(&z_diff(1)); // angle normalization
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise
  S(0, 0) += std_radr_ * std_radr_;
  S(1, 1) += std_radphi_ * std_radphi_;
  S(2, 2) += std_radrd_ * std_radrd_;

  // cross correlation matrix
  MatrixXd Tc(n_x_, n_z);
  Tc.fill(0.0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred; // residual
    NormalizePhi(&z_diff(1));// angle normalization
    VectorXd x_diff = Xsig_pred_.col(i) - x_; // state difference
    NormalizePhi(&x_diff(3)); // angle normalization
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred; // residual
  NormalizePhi(&z_diff(1)); // angle normalization
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  
  /**
   * linear update using lidar measurement
   */

  MatrixXd H_lidar(2, n_x_);
  H_lidar << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0;

  MatrixXd R_lidar(2, 2);
  R_lidar << std_laspx_ * std_laspx_, 0,
             0, std_laspy_ * std_laspy_;            

  VectorXd y = meas_package.raw_measurements_ - H_lidar * x_;
  MatrixXd S = H_lidar * P_ * H_lidar.transpose() + R_lidar;
  MatrixXd K = P_ * H_lidar.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_lidar) * P_;
}

void UKF::NormalizePhi(double *x) {

  // make sure phi is in the range (-pi, pi)
  while ((*x) > M_PI) (*x) -= 2. * M_PI;
  while ((*x) < -M_PI) (*x) += 2. * M_PI;
}