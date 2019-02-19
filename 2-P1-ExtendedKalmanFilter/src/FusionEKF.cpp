#include "FusionEKF.h"

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // lidar measurement covariance matrix
  ekf_.R_lidar_ << 0.0225, 0,
                   0, 0.0225;

  // radar measurement covariance matrix
  ekf_.R_radar_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;

  // measurement matrix for lidar update
  ekf_.H_lidar_ << 1, 0, 0, 0,
                   0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  const VectorXd &z = measurement_pack.raw_measurements_;

  /**
   * Initialization
   */
  if (!is_initialized_) {
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.x_ << z(0) * cos(z(1)), z(0) * sin(z(1)), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << z(0), z(1), 0, 0;
    }

	// state covariance matrix
	ekf_.P_ << 1, 0, 0, 0,
		       0, 1, 0, 0,
		       0, 0, 1, 0,
               0, 0, 0, 1;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e+6;
  previous_timestamp_ = measurement_pack.timestamp_;

  // process noise
  double noise_ax = 9;
  double noise_ay = 9;

  ekf_.Predict(dt, noise_ax, noise_ay);

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateRadar(z);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.UpdateLidar(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
