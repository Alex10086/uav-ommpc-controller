#pragma once

#include <vector>

#include "uav_ommpc_controller/dynamic.hpp"

namespace uav_ommpc_controller::reference
{
// Ascending coefficients in normalized local time s = t / duration.
struct Segment
{
  double duration;
  std::vector<double> x, y, z, yaw;
};

struct FlatSample
{
  Eigen::Vector3d p, v, a, j;
  double yaw;
  double yaw_rate;
};

class PolynomialReference
{
public:
  explicit PolynomialReference(
    std::vector<Segment> segments, bool hold_final = false,
    double gravity = dynamics::kGravity);

  double duration() const;
  // Individual sample outputs are unchanged on failure.
  bool sample_flat(double elapsed, FlatSample &out) const;
  bool sample(double elapsed, dynamics::State &state, dynamics::ControlInput &input) const;
  // Pre-sized N+1 states and N inputs, N >= 1. No resizing/allocation.
  // On failure the caller must discard potentially partially written buffers.
  bool sample_horizon(
    double elapsed, double dt, std::vector<dynamics::State> &states,
    std::vector<dynamics::ControlInput> &inputs) const;

private:
  std::vector<Segment> segments_;
  std::vector<double> ends_;
  bool hold_final_;
  double gravity_;
  FlatSample final_;
};
}  // namespace uav_ommpc_controller::reference
