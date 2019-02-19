#include "kalman_filter.h"

static const double div_eps = 1e-5;

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  P_ = MatrixXd(4, 4);
  R_radar_ = MatrixXd(3, 3);
  R_lidar_ = MatrixXd(2, 2);
  H_radar_ = MatrixXd(3, 4);
  H_lidar_ = MatrixXd(2, 4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict(const double dt, const double noise_ax, const double noise_ay) {  
  
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
        0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
        dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
        0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateLidar(const VectorXd &z) {

  VectorXd y = z - H_lidar_ * x_;
  MatrixXd S = H_lidar_ * P_ * H_lidar_.transpose() + R_lidar_;
  MatrixXd K = P_ * H_lidar_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_lidar_) * P_;
}

void KalmanFilter::UpdateRadar(const VectorXd &z) {

  // avoid division by zero
  double div = x_(0) * x_(0) + x_(1) * x_(1);
  if (fabs(div) < div_eps) return;

  VectorXd x_polar(3);
  Cartesian2Polar(x_, &x_polar);
  VectorXd y = z - x_polar;
  CalculateJacobian(x_, &H_radar_);
  MatrixXd S = H_radar_ * P_ * H_radar_.transpose() + R_radar_;
  MatrixXd K = P_ * H_radar_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_radar_) * P_;
}

void KalmanFilter::Cartesian2Polar(const VectorXd &x, VectorXd *x_polar) {
  // TODO: check for division by zero, normalize
  (*x_polar)(0) = sqrt(x(0) * x(0) + x(1) * x(1));
  (*x_polar)(1) = atan2(x(1), x(0));
  (*x_polar)(2) = (x(0) * x(2) + x(1) * x(3)) / (*x_polar)(0);
}

void KalmanFilter::CalculateJacobian(const VectorXd& x_state, MatrixXd *Hj) {

  // current state
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;

  // calculate the Jacobian
  *Hj << (px / c2), (py / c2), 0, 0,
        -(py / c1), (px / c1), 0, 0,
         py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
}