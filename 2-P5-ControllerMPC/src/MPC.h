#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
//#include "helpers.h"

using CppAD::AD;
using Eigen::VectorXd;
//typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

//typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC {
public:
  
   MPC();

   virtual ~MPC();

   // Solve the model given an initial state and polynomial coefficients.
   // Return the first actuations.
   std::vector<double> Solve(const Eigen::VectorXd &state, 
                             const Eigen::VectorXd &coeffs);

   // place to return solution
   //CppAD::ipopt::solve_result<CPPAD_TESTVECTOR(double)> solution_;
};

#endif  // MPC_H
