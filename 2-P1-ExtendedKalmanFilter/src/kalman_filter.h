#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Predicts the state and the state covariance using the process model
   */
  void Predict(const double dt, const double noise_ax, const double noise_ay);

  /**
   * Updates the state by using standard Kalman Filter equations (for Lidar)
   */
  void UpdateLidar(const VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations (for Radar)
   */
  void UpdateRadar(const VectorXd &z);

  /**
   * Calculate the Jacobian for radar update
   */
  void CalculateJacobian(const VectorXd& x_state, MatrixXd *Hj);

  /**
   * Convert cartesian state vector to polar
   */
  void Cartesian2Polar(const VectorXd &x, VectorXd *x_polar);

  /**
   * Normalize phi angle
   */
  void NormalizePhi(VectorXd *y);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix for lidar update and Jacobian for radar update
  MatrixXd H_lidar_;  
  MatrixXd H_radar_;

  // measurement covariance matrices
  MatrixXd R_lidar_;
  MatrixXd R_radar_;
};

#endif // KALMAN_FILTER_H_
