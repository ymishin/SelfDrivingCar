#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  first_measurement_done = false;
}

double PID::CalculateSteer(double cte) {
  
  if (!first_measurement_done) {
    p_error_ = cte;
    i_error_ = cte;
    d_error_ = cte;
    first_measurement_done = true;
    return 0.0;
  }
  
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;

  double steer = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
  return steer;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}