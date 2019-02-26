#include "MPC.h"

/**
 * TODO: Set the timestep length and duration
 */
size_t N = 50;
double dt = 0.05;

// This value was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the radius formed by the simulating the model
//   matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

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
     * TODO: implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable 
     *   values (state & actuators)
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */

    double ref_v = 35;

    fg[0] = 0;

    /*
     * Calculate objective
     */

    // Cross track error, heading error, velocity to reference error
    for (unsigned t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start_ + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start_ + t], 2);
      //fg[0] += CppAD::pow(vars[v_start_ + t] - ref_v, 2);
    }
    // Minimize the use of actuators
    for (unsigned t = 0; t < N - 1; t++) {

      fg[0] += CppAD::pow(vars[delta_start_ + t], 2);
      fg[0] += CppAD::pow(vars[a_start_ + t], 2);
    }
    // Minimize the value gap between sequential actuations
    for (unsigned t = 0; t < N - 2; t++) {
      fg[0] += CppAD::pow(vars[delta_start_ + t + 1] - vars[delta_start_ + t], 2);
      fg[0] += CppAD::pow(vars[a_start_ + t + 1] - vars[a_start_ + t], 2);
    }

    fg[1 + x_start_] = vars[x_start_];
    fg[1 + y_start_] = vars[y_start_];
    fg[1 + psi_start_] = vars[psi_start_];
    fg[1 + v_start_] = vars[v_start_];
    fg[1 + cte_start_] = vars[cte_start_];
    fg[1 + epsi_start_] = vars[epsi_start_];

    for (unsigned t = 1; t < N; ++t) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start_ + t];
      AD<double> y1 = vars[y_start_ + t];
      AD<double> psi1 = vars[psi_start_ + t];
      AD<double> v1 = vars[v_start_ + t];
      AD<double> cte1 = vars[cte_start_ + t];
      AD<double> epsi1 = vars[epsi_start_ + t];

      // The state at time t.
      AD<double> x0 = vars[x_start_ + t - 1];
      AD<double> y0 = vars[y_start_ + t - 1];
      AD<double> psi0 = vars[psi_start_ + t - 1];
      AD<double> v0 = vars[v_start_ + t - 1];
      AD<double> cte0 = vars[cte_start_ + t - 1];
      AD<double> epsi0 = vars[epsi_start_ + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start_ + t - 1];
      AD<double> a0 = vars[a_start_ + t - 1];

      AD<double> f0 = coeffs_[0] + coeffs_[1] * x0;
      AD<double> psides0 = CppAD::atan(coeffs_[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + x_start_ + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start_ + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start_ + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start_ + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start_ + t] =
        cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start_ + t] =
        epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
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

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars = 6 * N + 2 * (N - 1);
  
  /**
   * TODO: Set the number of constraints
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

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  vars[x_start] = state(0);
  vars[y_start] = state(1);
  vars[psi_start] = state(2);
  vars[v_start] = state(3);
  vars[cte_start] = state(4);
  vars[epsi_start] = state(5);
  
  /**
   * TODO: Set lower and upper limits for variables.
   */
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // ...
  for (unsigned i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  // lower and upper limits for steering value
  for (unsigned i = delta_start; i < delta_start + N - 1; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // lower and upper limits for throttle value
  for (unsigned i = a_start; i < a_start + N - 1; ++i) {
    vars_lowerbound[i] = -1.;
    vars_upperbound[i] = 1.;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0.;
    constraints_upperbound[i] = 0.;
  }
  constraints_lowerbound[x_start] = vars[x_start];
  constraints_upperbound[x_start] = vars[x_start];
  constraints_lowerbound[y_start] = vars[y_start];
  constraints_upperbound[y_start] = vars[y_start];
  constraints_lowerbound[psi_start] = vars[psi_start];
  constraints_upperbound[psi_start] = vars[psi_start];
  constraints_lowerbound[v_start] = vars[v_start];
  constraints_upperbound[v_start] = vars[v_start];
  constraints_lowerbound[cte_start] = vars[cte_start];
  constraints_upperbound[cte_start] = vars[cte_start];
  constraints_lowerbound[epsi_start] = vars[epsi_start];
  constraints_upperbound[epsi_start] = vars[epsi_start];

  // object that computes objective and constraints
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

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution_;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution_);

  // Check some of the solution values
  ok &= solution_.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution_.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * TODO: Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */

  // Return the first actuator values
  return { solution_.x[delta_start], solution_.x[a_start] };
}