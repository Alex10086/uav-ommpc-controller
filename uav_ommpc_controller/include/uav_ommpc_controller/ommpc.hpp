#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "uav_ommpc_controller/linearization.hpp"

namespace uav_ommpc_controller::ommpc
{
struct Settings
{
  int horizon = 20;
  double dt = 0.02;
  double gravity = dynamics::kGravity;
  linearization::StateError state_weights =
    (linearization::StateError() << 1000.0, 1000.0, 800.0, 20.0, 20.0, 20.0, 40.0, 40.0, 40.0).finished();
  linearization::StateError terminal_weights = 2.0 * state_weights;
  linearization::InputError input_weights{0.5, 1.2, 1.2, 0.6};
  linearization::InputError input_min{1.0, -3.0, -3.0, -1.5};
  linearization::InputError input_max{12.0, 3.0, 3.0, 1.5};
  int max_iterations = 400;
  double absolute_tolerance = 1e-5;
  double relative_tolerance = 1e-5;
};

// Paper notation: delta_X = M * delta_U + H * delta_x0, stacking x1..xN.
// Solver objective: 0.5 * delta_U^T * P * delta_U + q^T * delta_U.
struct CondensedQp
{
  Eigen::MatrixXd M;
  Eigen::MatrixXd H;
  Eigen::MatrixXd P;
  Eigen::VectorXd q;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
};

enum class Status { Solved, InvalidInput, ReferenceMismatch, SolverFailure };

struct Result
{
  Status status = Status::InvalidInput;
  std::optional<dynamics::ControlInput> command;  // Empty on every failure; no fallback command.
  int solver_status = 0;                         // Native OSQP status, when solved/attempted.
  int iterations = 0;
  double max_constraint_violation = 0.0;         // Original box constraints, accepted <= 1e-4.
};

// One instance per control loop; not thread-safe. Allocates workspace at construction.
// References: N+1 states and N inputs, sampled at dt, dynamically consistent to 1e-6
// per position/velocity/attitude norm, with inputs inside the configured box.
// NED/FRD, thrust acceleration in m/s^2, body rates in rad/s; no ROS or motor model.
class Controller
{
public:
  explicit Controller(const Settings &settings = Settings{});
  ~Controller();
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;

  Result solve(const dynamics::State &actual, const std::vector<dynamics::State> &reference,
    const std::vector<dynamics::ControlInput> &feedforward);

  // Inspection views owned by this controller, overwritten by subsequent solve calls.
  // problem() is valid after a solved call; solution views contain NaNs after failure.
  const CondensedQp &problem() const;
  const Eigen::VectorXd &input_errors() const;
  const Eigen::VectorXd &predicted_errors() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace uav_ommpc_controller::ommpc
