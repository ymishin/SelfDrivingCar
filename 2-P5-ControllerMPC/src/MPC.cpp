#include "MPC.h"

/**
 * Timestep and duration
 */
static const double dt = 0.1;
static const size_t N = 10;

/**
 * This value was obtained by measuring the radius formed by running the vehicle in the
 *   simulator around in a circle with a constant steering angle and velocity on
 *   a flat terrain.
 * Lf was tuned until the radius formed by the simulating the model
 *   matched the previous radius.
 * This is the length from front to CoG that has a similar radius.
 */
static const double Lf = 2.67;

/**
 * Reference velocity
 */
static const double ref_v = 35.;

class FG_eval {

public:

  FG_eval(
    VectorXd coeffs, 
    unsigned x_start,
    unsigned y_start,
    unsigned psi_start,
    unsigned v_start,
    unsigned cte_start,
    unsigned epsi_start,
    unsigned delta_start,
    unsigned a_start)
    :
    coeffs_(coeffs),
    x_start_(x_start),
    y_start_(y_start),
    psi_start_(psi_start),
    v_start_(v_start),
    cte_start_(cte_start),
    epsi_start_(epsi_start),
    delta_start_(delta_start),
    a_start_(a_start)
  {}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    /**
     * `fg` is a vector of the cost constraints
     * `vars` is a vector of variable values (state & actuators)
     */

    /*
     * Calculate objective
     */

    fg[0] = 0;

    // Cross track error, heading error, velocity to reference error
    for (unsigned i = 0; i < N; ++i) {
      fg[0] += 2.e+3 * CppAD::pow(vars[cte_start_ + i], 2);
      fg[0] += 2.e+3 * CppAD::pow(vars[epsi_start_ + i], 2);
      fg[0] += CppAD::pow(vars[v_start_ + i] - ref_v, 2);
    }
    
    // Minimize the use of actuators
    for (unsigned i = 0; i < N - 1; ++i) {

      fg[0] += 5. * CppAD::pow(vars[delta_start_ + i], 2);
      fg[0] += 5. * CppAD::pow(vars[a_start_ + i], 2);
    }
    
    // Minimize the value gap between sequential actuations
    for (unsigned i = 0; i < N - 2; ++i) {
      fg[0] += 2.e+2 * CppAD::pow(vars[delta_start_ + i + 1] - vars[delta_start_ + i], 2);
      fg[0] += 1.e+1 * CppAD::pow(vars[a_start_ + i + 1] - vars[a_start_ + i], 2);
    }

    /*
     * Calculate constraints 
     */

    fg[x_start_ + 1] = vars[x_start_];
    fg[y_start_ + 1] = vars[y_start_];
    fg[psi_start_ + 1] = vars[psi_start_];
    fg[v_start_ + 1] = vars[v_start_];
    fg[cte_start_ + 1] = vars[cte_start_];
    fg[epsi_start_ + 1] = vars[epsi_start_];

    for (unsigned i = 1; i < N; ++i) {
      
      // The state at time t + 1
      AD<double> x1 = vars[x_start_ + i];
      AD<double> y1 = vars[y_start_ + i];
      AD<double> psi1 = vars[psi_start_ + i];
      AD<double> v1 = vars[v_start_ + i];
      AD<double> cte1 = vars[cte_start_ + i];
      AD<double> epsi1 = vars[epsi_start_ + i];

      // The state at time t
      AD<double> x0 = vars[x_start_ + i - 1];
      AD<double> y0 = vars[y_start_ + i - 1];
      AD<double> psi0 = vars[psi_start_ + i - 1];
      AD<double> v0 = vars[v_start_ + i - 1];
      AD<double> cte0 = vars[cte_start_ + i - 1];
      AD<double> epsi0 = vars[epsi_start_ + i - 1];

      // Actuators at time t
      AD<double> delta0 = vars[delta_start_ + i - 1];
      AD<double> a0 = vars[a_start_ + i - 1];

      AD<double> f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2] * x0 * x0 + coeffs_[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs_[1] + 2 * coeffs_[2] * x0 + 3 * coeffs_[3] * x0 * x0);

      // Constraints
      fg[x_start_ + 1 + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[y_start_ + 1 + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[psi_start_ + 1 + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[v_start_ + 1 + i] = v1 - (v0 + a0 * dt);
      fg[cte_start_ + 1 + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[epsi_start_ + 1 + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }

private:

  VectorXd coeffs_; // fitted polynomial coefficients
  unsigned x_start_;
  unsigned y_start_;
  unsigned psi_start_;
  unsigned v_start_;
  unsigned cte_start_;
  unsigned epsi_start_;
  unsigned delta_start_;
  unsigned a_start_;
};

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  
  bool ok = true;
  
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * Number of model variables (both states and inputs)
   */
  size_t n_vars = 6 * N + 2 * (N - 1);
  
  /**
   * Number of constraints
   */
  size_t n_constraints = 6 * N;

  unsigned x_start = 0;
  unsigned y_start = x_start + N;
  unsigned psi_start = y_start + N;
  unsigned v_start = psi_start + N;
  unsigned cte_start = v_start + N;
  unsigned epsi_start = cte_start + N;
  unsigned delta_start = epsi_start + N;
  unsigned a_start = delta_start + N - 1;

  /**
   * Initial values of the independent variables
   */
  Dvector vars(n_vars);
  for (unsigned i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  // Initial state
  vars[x_start] = state(0);
  vars[y_start] = state(1);
  vars[psi_start] = state(2);
  vars[v_start] = state(3);
  vars[cte_start] = state(4);
  vars[epsi_start] = state(5);
  
  /**
   * Lower and upper limits for variables
   */
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // default limits for all variables
  for (unsigned i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  // lower and upper limits for steering
  for (unsigned i = delta_start; i < delta_start + N - 1; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // lower and upper limits for throttle
  for (unsigned i = a_start; i < a_start + N - 1; ++i) {
    vars_lowerbound[i] = -1.;
    vars_upperbound[i] = 1.;
  }

  /**
   * Lower and upper limits for the constraints
   */
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0.;
    constraints_upperbound[i] = 0.;
  }
  constraints_lowerbound[x_start] = state(0);
  constraints_upperbound[x_start] = state(0);
  constraints_lowerbound[y_start] = state(1);
  constraints_upperbound[y_start] = state(1);
  constraints_lowerbound[psi_start] = state(2);
  constraints_upperbound[psi_start] = state(2);
  constraints_lowerbound[v_start] = state(3);
  constraints_upperbound[v_start] = state(3);
  constraints_lowerbound[cte_start] = state(4);
  constraints_upperbound[cte_start] = state(4);
  constraints_lowerbound[epsi_start] = state(5);
  constraints_upperbound[epsi_start] = state(5);

  // Object that computes objective and constraints
  FG_eval fg_eval(
    coeffs,
    x_start,
    y_start,
    psi_start,
    v_start,
    cte_start,
    epsi_start,
    delta_start,
    a_start);

  // Options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // Solve the problem
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Actuator values
  std::vector<double> result;
  double delta_act = (solution.x[delta_start] + solution.x[delta_start + 1] + solution.x[delta_start + 2]) / 3.;  
  double a_act = (solution.x[a_start] + solution.x[a_start + 1] + solution.x[a_start + 2]) / 3.;  
  delta_act /= 0.436332; // normalize
  result.push_back(delta_act);
  result.push_back(a_act);

  // Predicted trajectory
  for (unsigned i = 0; i < N - 1; ++i) {
    result.push_back(solution.x[x_start + 1 + i]);
    result.push_back(solution.x[y_start + 1 + i]);
  }

  return result;
}