#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  first_measurement_done = false;
}

double PID::TotalError(double cte) {
  
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
  double total = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
  
  return total;
}