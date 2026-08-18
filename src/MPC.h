#ifndef MPC_H
#define MPC_H

#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  explicit MPC(const std::string &config_path = "mpc_config.json");

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state,
                            const Eigen::VectorXd &coeffs);
};

#endif  // MPC_H
