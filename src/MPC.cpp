#include "MPC.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

static const size_t N  = 15;
static const double dt = 0.04;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
static const double Lf = 2.67;

// reference velocity
static const double ref_v = 100;

// starting indices
static const size_t x_start     = 0;
static const size_t y_start     = x_start + N;
static const size_t psi_start   = y_start + N;
static const size_t v_start     = psi_start + N;
static const size_t cte_start   = v_start + N;
static const size_t epsi_start  = cte_start + N;
static const size_t delta_start = epsi_start + N - 1;
static const size_t a_start     = delta_start + N - 1;

// weights
static const double delta_weight  = 1.0;
static const double a_weight      = 4.0;

// ddelta and da weights scale as a function of reference velocity
static const double ddelta_weight = pow(ref_v, 2.2);
static const double da_weight     = ddelta_weight / 10.0;

// bounds
static const double non_actuator_bound = 1.0e19;
static const double delta_bound = 0.436332;  // 25 degrees in radians
static const double accel_bound = 1.0;
static const double constraint_bound = 0.0;

class FG_eval {
 public:
  VectorXd coeffs;

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  explicit FG_eval(VectorXd coeffs) {
    this->coeffs = coeffs;
  }

  void operator()(ADvector &fg, const ADvector &vars) {
    // store the cost in the first element of `fg`.
    fg[0] = 0;

    // reference state cost
    for (int t = 0; t < N; ++t) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // cost in proportion to control inputs
    for (int t = 0; t < N - 1; ++t) {
      fg[0] += delta_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_weight * CppAD::pow(vars[a_start + t], 2);
    }

    // cost in proportion to change in control inputs
    for (int t = 0; t < N - 2; ++t) {
      fg[0] += ddelta_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += da_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    // add 1 to each of the starting indices due to cost being located at 0
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t0 = 0, t1 = 1; t1 < N; ++t0, ++t1) {
      // The state at time t = 0
      AD<double> x0    = vars[x_start + t0];
      AD<double> y0    = vars[y_start + t0];
      AD<double> psi0  = vars[psi_start + t0];
      AD<double> v0    = vars[v_start + t0];
      AD<double> epsi0 = vars[epsi_start + t0];

      // The state at time t + 1
      AD<double> x1    = vars[x_start + t1];
      AD<double> y1    = vars[y_start + t1];
      AD<double> psi1  = vars[psi_start + t1];
      AD<double> v1    = vars[v_start + t1];
      AD<double> cte1  = vars[cte_start + t1];
      AD<double> epsi1 = vars[epsi_start + t1];

      // Only consider the actuation at time t0
      AD<double> delta0 = vars[delta_start + t0];
      AD<double> a0     = vars[a_start + t0];

      // Calculate the fit path and its derivative at position x0
      AD<double> f0 = 0.0;
      AD<double> df0 = 0.0;
      for (int i = 0; i < coeffs.size(); ++i) {
        f0 += coeffs[i] * CppAD::pow(x0, i);

        // zero order taylor coefficients are not allowed
        if (i > 0) {
          df0 += i * coeffs[i] * CppAD::pow(x0, i-1);
        }
      }

      // Derivative of the fit path at x0
      AD<double> psi_des0 = CppAD::atan(df0);

      // motion model
      fg[1 + x_start + t1]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t1]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t1]  = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t1]    = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t1]  = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t1] = epsi1 - (psi0 - psi_des0 + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::solve(VectorXd state, VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // number of constraints
  size_t n_constraints = N * 6;

  // number of model variables (includes both states and inputs).
  size_t n_vars = n_constraints + (N - 1) * 2;

  // Initial value of the independent variables.
  Dvector vars(n_vars);
  for (int t = 0; t < n_vars; ++t) {
    vars[t] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set the initial variable values
  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;

  // non-actuator upper and lower bounds
  for (size_t t = 0; t < delta_start; ++t) {
    vars_lowerbound[t] = -non_actuator_bound;
    vars_upperbound[t] =  non_actuator_bound;
  }

  // delta upper and lower bounds
  for (size_t t = delta_start; t < a_start; ++t) {
    vars_lowerbound[t] = -delta_bound;
    vars_upperbound[t] =  delta_bound;
  }

  // acceleration/decceleration upper and lower bounds
  for (size_t t = a_start; t < n_vars; ++t) {
    vars_lowerbound[t] = -accel_bound;
    vars_upperbound[t] =  accel_bound;
  }

  // upper and lower bounds for the constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t t = 0; t < n_constraints; ++t) {
    constraints_lowerbound[t] = constraint_bound;
    constraints_upperbound[t] = constraint_bound;
  }

  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options,
    vars,
    vars_lowerbound,
    vars_upperbound,
    constraints_lowerbound,
    constraints_upperbound,
    fg_eval,
    solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost: " << cost << "; Ok: " << ok << std::endl;

  vector<double> result = {
    solution.x[delta_start],
    solution.x[a_start]
  };

  for (size_t t = 0; t < N; ++t) {
    result.push_back(solution.x[x_start + t]);
  }
  for (size_t t = 0; t < N; ++t) {
    result.push_back(solution.x[y_start + t]);
  }

  return result;
}
