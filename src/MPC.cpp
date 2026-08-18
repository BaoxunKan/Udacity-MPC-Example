#include "MPC.h"

#include <cppad/ipopt/solve.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "helpers.h"
#include "json.hpp"

using CppAD::AD;
using Eigen::VectorXd;
using nlohmann::json;

size_t N;
double dt;
double Lf;

std::size_t x_start;
std::size_t y_start;
std::size_t psi_start;
std::size_t v_start;
std::size_t cte_start;
std::size_t epsi_start;
std::size_t delta_start;
std::size_t a_start;

double dCost_factor_cte;
double dCost_factor_epsi;
double dCost_factor_v;
double dCost_factor_steering;
double dCost_factor_steering_quartic;
double dCost_factor_throttle;
double dCost_factor_steering_slope;
double dCost_factor_throttle_slope;

double ref_v;

namespace {
void UpdateVariableStarts() {
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;
}

void LoadConfig(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    std::cerr << "Failed to open config file: " << path << std::endl;
    std::exit(1);
  }

  json cfg;
  in >> cfg;

  N = cfg.at("N").get<size_t>();
  dt = cfg.at("dt").get<double>();
  Lf = cfg.at("Lf").get<double>();
  ref_v = cfg.at("ref_v_mph").get<double>() * 1609.34 / 3600.0;

  const json& cost = cfg.at("cost");
  dCost_factor_cte = cost.at("cte").get<double>();
  dCost_factor_epsi = cost.at("epsi").get<double>();
  dCost_factor_v = cost.at("v").get<double>();
  dCost_factor_steering = cost.at("steering").get<double>();
  dCost_factor_steering_quartic = cost.at("steering_quartic").get<double>();
  dCost_factor_throttle = cost.at("throttle").get<double>();
  dCost_factor_steering_slope = cost.at("steering_slope").get<double>();
  dCost_factor_throttle_slope = cost.at("throttle_slope").get<double>();

  UpdateVariableStarts();

  std::cout << "Loaded MPC config from " << path << ": N=" << N << " dt=" << dt
            << " ref_v=" << ref_v << " m/s" << std::endl;
}
}  // namespace

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * TODO: implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable
     *   values (state & actuators)
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */

    // The part of the cost based on the reference state.
    fg[0] = 0;

    // Minimize the squared error of the CTE and EPE.
    // CTE is the error in the lateral position.
    // EPE is the error in the orientation.
    // V is the velocity.
    for (size_t t = 0; t < N; t++) {
      fg[0] += dCost_factor_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += dCost_factor_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += dCost_factor_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the squared error of the steering and throttle.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += dCost_factor_steering * CppAD::pow(vars[delta_start + t], 2);
      fg[0] +=
          dCost_factor_steering_quartic * CppAD::pow(vars[delta_start + t], 4);
      fg[0] += dCost_factor_throttle * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += dCost_factor_steering_slope *
               CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += dCost_factor_throttle_slope *
               CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Constraints
    // initial state
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];    // cte at time t
    fg[1 + epsi_start] = vars[epsi_start];  // epsi at time t

    for (size_t t = 1; t < N; t++) {
      // The state at time t.
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t-1.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = cppadPolyeval(coeffs, x0, false);

      AD<double> psi_des = CppAD::atan(cppadPolyeval(coeffs, x0, true));

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psi_des) + v0 / Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(const std::string& config_path) { LoadConfig(config_path); }
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd& state, const VectorXd& coeffs) {
  bool ok = true;
  using Dvector = CPPAD_TESTVECTOR(double);

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  /**
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars =
      N * 6 +
      (N - 1) *
          2;  // 6 states for each time step, 2 actuators for each time step
  /**
   * TODO: Set the number of constraints
   */
  size_t n_constraints = N * 6;  // 6 states for each time step

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /**
   * TODO: Set lower and upper limits for variables.
   */
  // Set the initial values of the variables
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Set the lower and upper limits for the variables
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1e19;
    vars_upperbound[i] = 1e19;
  }
  // Set the lower and upper limits for the steering angle to -25 degrees and 25
  // degrees
  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Set the lower and upper limits for the throttle to -1 and 1
  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set the lower and upper limits for the initial state
  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
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

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * TODO: Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */
  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i = 0; i < N; ++i) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}