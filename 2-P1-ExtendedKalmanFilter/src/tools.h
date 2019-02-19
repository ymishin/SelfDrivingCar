#ifndef TOOLS_H_
#define TOOLS_H_

#include "Eigen/Dense"
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, 
                                const vector<VectorXd> &ground_truth);
};

#endif  // TOOLS_H_
