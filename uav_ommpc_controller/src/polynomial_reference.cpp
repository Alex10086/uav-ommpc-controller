#include "uav_ommpc_controller/polynomial_reference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace uav_ommpc_controller::reference
{
namespace
{
constexpr double kContinuityTolerance = 1e-6;

// Simultaneous Horner evaluation of value and first three s derivatives.
// Divide successively by duration to convert to physical-time derivatives;
// do not form inverse-duration powers (which can overflow for constant axes).
std::array<double, 4> evaluate_axis(const std::vector<double> &c, double s, double duration)
{
  std::array<double, 4> d{};
  for (auto it = c.rbegin(); it != c.rend(); ++it)
  {
    d[3] = d[3] * s + 3.0 * d[2];
    d[2] = d[2] * s + 2.0 * d[1];
    d[1] = d[1] * s + d[0];
    d[0] = d[0] * s + *it;
  }
  for (std::size_t order = 1; order < d.size(); ++order)
    for (std::size_t power = 0; power < order; ++power)
      d[order] /= duration;
  return d;
}

FlatSample evaluate(const Segment &segment, double s)
{
  const auto x = evaluate_axis(segment.x, s, segment.duration);
  const auto y = evaluate_axis(segment.y, s, segment.duration);
  const auto z = evaluate_axis(segment.z, s, segment.duration);
  const auto yaw = evaluate_axis(segment.yaw, s, segment.duration);
  return {{x[0], y[0], z[0]}, {x[1], y[1], z[1]},
    {x[2], y[2], z[2]}, {x[3], y[3], z[3]}, yaw[0], yaw[1]};
}

bool finite(const FlatSample &s)
{
  return s.p.allFinite() && s.v.allFinite() && s.a.allFinite() && s.j.allFinite() &&
    std::isfinite(s.yaw) && std::isfinite(s.yaw_rate);
}

bool continuous(const FlatSample &left, const FlatSample &right)
{
  return (left.p - right.p).stableNorm() <= kContinuityTolerance &&
    (left.v - right.v).stableNorm() <= kContinuityTolerance &&
    (left.a - right.a).stableNorm() <= kContinuityTolerance &&
    (left.j - right.j).stableNorm() <= kContinuityTolerance &&
    std::abs(left.yaw - right.yaw) <= kContinuityTolerance &&
    std::abs(left.yaw_rate - right.yaw_rate) <= kContinuityTolerance;
}

bool stationary(const FlatSample &s)
{
  return s.v.stableNorm() <= kContinuityTolerance &&
    s.a.stableNorm() <= kContinuityTolerance &&
    s.j.stableNorm() <= kContinuityTolerance &&
    std::abs(s.yaw_rate) <= kContinuityTolerance;
}
}  // namespace

PolynomialReference::PolynomialReference(
  std::vector<Segment> segments, bool hold_final, double gravity)
: segments_(std::move(segments)), hold_final_(hold_final), gravity_(gravity)
{
  if (segments_.empty() || segments_.size() > 100 || !std::isfinite(gravity_) || gravity_ <= 0.0)
    throw std::invalid_argument("Invalid polynomial segment count or gravity");
  ends_.reserve(segments_.size());
  double end = 0.0;
  std::optional<FlatSample> previous;
  for (const auto &segment : segments_)
  {
    if (!std::isfinite(segment.duration) || segment.duration <= 0.0)
      throw std::invalid_argument("Polynomial duration must be finite and positive");
    for (const auto *axis : {&segment.x, &segment.y, &segment.z, &segment.yaw})
    {
      if (axis->empty() || axis->size() > 10 ||
        !std::all_of(axis->begin(), axis->end(), [](double c) {return std::isfinite(c);}))
        throw std::invalid_argument("Invalid polynomial coefficient array");
    }
    const double next_end = end + segment.duration;
    if (!std::isfinite(next_end) || next_end <= end)
      throw std::invalid_argument("Polynomial cumulative duration is not representable");
    const auto start = evaluate(segment, 0.0);
    const auto finish = evaluate(segment, 1.0);
    if (!finite(start) || !finite(finish))
      throw std::invalid_argument("Nonfinite polynomial endpoint derivatives");
    if (previous && !continuous(*previous, start))
      throw std::invalid_argument("Polynomial requires C3 position and C1 unwrapped yaw continuity");
    ends_.push_back(next_end);
    end = next_end;
    previous = finish;
  }
  final_ = previous.value();  // A nonempty, validated segment sequence always sets this.
  if (hold_final_ && !stationary(final_))
    throw std::invalid_argument("hold_final requires stationary final derivatives");
}

double PolynomialReference::duration() const
{
  return ends_.back();
}

bool PolynomialReference::sample_flat(double elapsed, FlatSample &out) const
{
  if (!std::isfinite(elapsed) || elapsed < 0.0) return false;
  if (elapsed > duration())
  {
    if (!hold_final_) return false;
    FlatSample held = final_;
    held.v.setZero();
    held.a.setZero();
    held.j.setZero();
    held.yaw_rate = 0.0;
    out = held;
    return true;
  }
  const auto it = std::upper_bound(ends_.begin(), ends_.end(), elapsed);
  const std::size_t index = it == ends_.end() ? segments_.size() - 1 :
    static_cast<std::size_t>(it - ends_.begin());
  const double start = index == 0 ? 0.0 : ends_[index - 1];
  const double s = elapsed == duration() ? 1.0 :
    (elapsed - start) / segments_[index].duration;
  const FlatSample result = evaluate(segments_[index], s);
  if (!finite(result)) return false;
  out = result;
  return true;
}

bool PolynomialReference::sample(
  double elapsed, dynamics::State &state, dynamics::ControlInput &input) const
{
  FlatSample flat;
  if (!sample_flat(elapsed, flat)) return false;
  const Eigen::Vector3d force = gravity_ * Eigen::Vector3d::UnitZ() - flat.a;
  const double thrust = force.stableNorm();
  if (!std::isfinite(thrust) || thrust <= 1e-6) return false;
  const Eigen::Vector3d b3 = force / thrust;
  if (b3.z() <= 1e-3) return false;
  const Eigen::Vector3d ypsi(-std::sin(flat.yaw), std::cos(flat.yaw), 0.0);
  const Eigen::Vector3d ypsi_dot = flat.yaw_rate *
    Eigen::Vector3d(-std::cos(flat.yaw), -std::sin(flat.yaw), 0.0);
  const Eigen::Vector3d h = ypsi.cross(b3);
  const double h_norm = h.stableNorm();
  if (!std::isfinite(h_norm) || h_norm <= 1e-6) return false;
  const Eigen::Vector3d b1 = h / h_norm;
  const Eigen::Vector3d b2 = b3.cross(b1);
  const Eigen::Vector3d b3_dot = (-flat.j + b3 * b3.dot(flat.j)) / thrust;
  const Eigen::Vector3d h_dot = ypsi_dot.cross(b3) + ypsi.cross(b3_dot);
  const Eigen::Vector3d b1_dot = (h_dot - b1 * b1.dot(h_dot)) / h_norm;
  const Eigen::Vector3d b2_dot = b3_dot.cross(b1) + b3.cross(b1_dot);
  dynamics::State result;
  result.p = flat.p;
  result.v = flat.v;
  result.R.col(0) = b1;
  result.R.col(1) = b2;
  result.R.col(2) = b3;
  Eigen::Matrix3d r_dot;
  r_dot.col(0) = b1_dot;
  r_dot.col(1) = b2_dot;
  r_dot.col(2) = b3_dot;
  const Eigen::Matrix3d body_derivative = result.R.transpose() * r_dot;
  dynamics::ControlInput control;
  control.thrust_acceleration = thrust;
  control.body_rates = 0.5 * Eigen::Vector3d(
    body_derivative(2, 1) - body_derivative(1, 2),
    body_derivative(0, 2) - body_derivative(2, 0),
    body_derivative(1, 0) - body_derivative(0, 1));
  if (!result.R.allFinite() || !control.body_rates.allFinite()) return false;
  state = result;
  input = control;
  return true;
}

bool PolynomialReference::sample_horizon(
  double elapsed, double dt, std::vector<dynamics::State> &states,
  std::vector<dynamics::ControlInput> &inputs) const
{
  if (inputs.empty() || states.empty() || states.size() - 1 != inputs.size() ||
    !std::isfinite(elapsed) || elapsed < 0.0 || !std::isfinite(dt) || dt <= 0.0)
    return false;
  const double last = elapsed + static_cast<double>(inputs.size()) * dt;
  if (!std::isfinite(last) || (!hold_final_ && last > duration())) return false;
  for (std::size_t k = 0; k < states.size(); ++k)
  {
    dynamics::ControlInput terminal_input;
    auto &input = k < inputs.size() ? inputs[k] : terminal_input;
    if (!sample(elapsed + static_cast<double>(k) * dt, states[k], input)) return false;
  }
  return true;
}
}  // namespace uav_ommpc_controller::reference
