#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.empty() || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean and squared root
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}