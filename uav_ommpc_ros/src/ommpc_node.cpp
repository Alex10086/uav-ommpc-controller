#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <uav_ommpc_interfaces/msg/polynomial_trajectory.hpp>
#include <uav_ommpc_interfaces/msg/thrust_rates.hpp>
#include <uav_ommpc_controller/ommpc.hpp>

#include "ommpc_inputs.hpp"

namespace
{
namespace dynamics = uav_ommpc_controller::dynamics;
namespace mpc = uav_ommpc_controller::ommpc;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;
using uav_ommpc_interfaces::msg::PolynomialTrajectory;
using uav_ommpc_interfaces::msg::ThrustRates;
using Clock = std::chrono::steady_clock;

class OmmpcNode final : public rclcpp::Node
{
public:
  OmmpcNode() : Node("ommpc_node")
  {
    settings_ = read_settings();
    odometry_timeout_ = timeout_parameter("odometry_timeout", 0.2);
    status_timeout_ = timeout_parameter("status_timeout", 1.5);
    reference_timeout_ = timeout_parameter("reference_timeout", 0.5);
    controller_ = std::make_unique<mpc::Controller>(settings_);
    period_ = std::chrono::nanoseconds(std::llround(settings_.dt * 1e9));
    reference_states_.resize(settings_.horizon + 1);
    reference_inputs_.resize(settings_.horizon);

    const auto telemetry = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    output_pub_ = create_publisher<ThrustRates>("/uav/control_input", reliable);
    odometry_sub_ = create_subscription<VehicleOdometry>("/uav/state", telemetry,
      [this](const VehicleOdometry &message) {receive_odometry(message);});
    status_sub_ = create_subscription<VehicleStatus>("/uav/status", telemetry,
      [this](const VehicleStatus &message) {receive_status(message);});
    reference_sub_ = create_subscription<PolynomialTrajectory>("/uav/reference_polynomial", reliable,
      [this](PolynomialTrajectory::ConstSharedPtr message) {receive_reference(*message);});
    timer_ = create_wall_timer(period_, [this]() {control_step();});
    RCLCPP_INFO(get_logger(),
      "OMMPC ready: N=%d, dt=%.6f s; waiting for odometry/status/reference; no automatic arm or mode change",
      settings_.horizon, settings_.dt);
  }

private:
  template<typename T>
  T parameter(const std::string &name, const T &value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;  // Cached settings cannot silently diverge from ROS parameters.
    return declare_parameter<T>(name, value, descriptor);
  }

  template<int Size>
  void vector_parameter(const std::string &name, Eigen::Matrix<double, Size, 1> &value)
  {
    const auto values = parameter(name, std::vector<double>(value.data(), value.data() + Size));
    if (values.size() != Size) throw std::invalid_argument(name + " has the wrong length");
    for (int i = 0; i < Size; ++i) value[i] = values[i];
  }

  double timeout_parameter(const std::string &name, double default_value)
  {
    const double value = parameter(name, default_value);
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument(name + " must be finite and positive");
    return value;
  }

  mpc::Settings read_settings()
  {
    mpc::Settings s;
    const auto horizon = parameter<std::int64_t>("horizon", s.horizon);
    const auto iterations = parameter<std::int64_t>("max_iterations", s.max_iterations);
    if (horizon < 1 || horizon > 100 || iterations < 1 || iterations > std::numeric_limits<int>::max())
      throw std::invalid_argument("horizon must be in [1,100] and max_iterations a positive int");
    s.horizon = static_cast<int>(horizon);
    s.max_iterations = static_cast<int>(iterations);
    s.dt = parameter("dt", s.dt);
    if (!std::isfinite(s.dt) || s.dt < 1e-6 || s.dt > 1.0)
      throw std::invalid_argument("dt must be in [1e-6,1] seconds for the ROS timer");
    s.gravity = parameter("gravity", s.gravity);
    vector_parameter("state_weights", s.state_weights);
    s.terminal_weights = 2.0 * s.state_weights;
    vector_parameter("terminal_weights", s.terminal_weights);
    vector_parameter("input_weights", s.input_weights);
    vector_parameter("input_min", s.input_min);
    vector_parameter("input_max", s.input_max);
    s.absolute_tolerance = parameter("absolute_tolerance", s.absolute_tolerance);
    s.relative_tolerance = parameter("relative_tolerance", s.relative_tolerance);
    s.max_position_defect = parameter("max_position_defect", s.max_position_defect);
    s.max_velocity_defect = parameter("max_velocity_defect", s.max_velocity_defect);
    s.max_attitude_defect = parameter("max_attitude_defect", s.max_attitude_defect);
    return s;  // Controller validates the remaining numerical settings.
  }

  void receive_odometry(const VehicleOdometry &message)
  {
    if (message.timestamp_sample == 0 || message.timestamp_sample > message.timestamp)
    {
      odometry_at_.reset();
      return;
    }
    // PX4 boot timestamps are not ROS time. Use them only to reject replay/frozen samples.
    if (message.timestamp_sample <= last_sample_) return;
    last_sample_ = message.timestamp_sample;
    if (reset_counter_ && *reset_counter_ != message.reset_counter)
    {
      trajectory_.reset();
      RCLCPP_WARN(get_logger(), "Estimator reset: waiting for a new reference in the current local frame");
    }
    reset_counter_ = message.reset_counter;
    if (!uav_ommpc_ros::decode_odometry(message, actual_))
    {
      odometry_at_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected odometry frame or nonfinite/invalid pose");
      return;
    }
    odometry_at_ = Clock::now();
    // Pair the measured state with ROS receipt time, never PX4 boot time.
    odometry_ros_ns_ = get_clock()->now().nanoseconds();
  }

  void receive_status(const VehicleStatus &message)
  {
    if (message.timestamp == 0)
    {
      status_at_.reset();
      return;
    }
    if (message.timestamp <= last_status_) return;
    last_status_ = message.timestamp;
    status_at_ = Clock::now();
    status_healthy_ = !message.failsafe && !message.in_transition_mode &&
      message.vehicle_type == VehicleStatus::VEHICLE_TYPE_ROTARY_WING &&
      message.nav_state != VehicleStatus::NAVIGATION_STATE_TERMINATION;
    // Do not require Offboard/armed: the bridge needs fresh inputs to prime its heartbeat.
  }

  void receive_reference(const PolynomialTrajectory &message)
  {
    trajectory_.reset();  // A malformed replacement must not silently keep an old plan active.
    try
    {
      trajectory_ = uav_ommpc_ros::decode_polynomial(message, reference_start_ns_, settings_.gravity);
      reference_at_ = Clock::now();
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejected polynomial reference: %s", error.what());
    }
  }

  static bool fresh(const std::optional<Clock::time_point> &at, double timeout, Clock::time_point now)
  {
    return at && now - *at <= std::chrono::duration<double>(timeout);
  }

  std::optional<double> active_reference(Clock::time_point steady_now, std::int64_t ros_now_ns)
  {
    if (ros_now_ns < last_ros_ns_)
    {
      trajectory_.reset();
      RCLCPP_WARN(get_logger(), "ROS clock moved backwards: reference invalidated");
    }
    last_ros_ns_ = ros_now_ns;
    if (!fresh(odometry_at_, odometry_timeout_, steady_now) ||
      !fresh(status_at_, status_timeout_, steady_now) || !status_healthy_ || !trajectory_ ||
      !fresh(reference_at_, reference_timeout_, steady_now)) return std::nullopt;
    const auto now_elapsed = uav_ommpc_ros::reference_elapsed(ros_now_ns, reference_start_ns_);
    const auto anchor_elapsed = uav_ommpc_ros::reference_elapsed(odometry_ros_ns_, reference_start_ns_);
    if (!now_elapsed || !anchor_elapsed || odometry_ros_ns_ > ros_now_ns) return std::nullopt;
    dynamics::State state;
    dynamics::ControlInput input;
    // Validate both current geometry and the measured state's prediction anchor.
    if (!trajectory_->sample(*now_elapsed, state, input) ||
      !trajectory_->sample(*anchor_elapsed, state, input)) return std::nullopt;
    return anchor_elapsed;
  }

  void control_step()
  {
    const auto started = Clock::now();
    const auto elapsed = active_reference(started, get_clock()->now().nanoseconds());
    if (!elapsed)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Control paused: need fresh valid odometry/status and a current full reference horizon");
      return;
    }
    if (!trajectory_->sample_horizon(*elapsed, settings_.dt, reference_states_, reference_inputs_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Control paused: polynomial cannot supply a valid full reference horizon");
      return;
    }
    const auto result = controller_->solve(actual_, reference_states_, reference_inputs_);
    if (result.status != mpc::Status::Solved || !result.command)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "OMMPC produced no command: status=%d, OSQP=%d, iterations=%d",
        static_cast<int>(result.status), result.solver_status, result.iterations);
      return;
    }
    const auto finished = Clock::now();
    // Ordinary reference sample boundary crossings are allowed. Bound solve
    // latency and recheck the lease/telemetry/domain, not a floor-grid index.
    if (finished - started > period_ ||
      !active_reference(finished, get_clock()->now().nanoseconds()))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Discarded late or expired OMMPC solution");
      return;
    }
    output_.thrust_acceleration = result.command->thrust_acceleration;
    for (std::size_t axis = 0; axis < 3; ++axis) output_.body_rates[axis] = result.command->body_rates[axis];
    output_pub_->publish(output_);
  }

  mpc::Settings settings_;
  std::unique_ptr<mpc::Controller> controller_;
  std::chrono::nanoseconds period_{};
  double odometry_timeout_{}, status_timeout_{}, reference_timeout_{};
  dynamics::State actual_;
  std::vector<dynamics::State> reference_states_;
  std::vector<dynamics::ControlInput> reference_inputs_;
  std::optional<Clock::time_point> odometry_at_, status_at_, reference_at_;
  std::uint64_t last_sample_{}, last_status_{};
  std::optional<std::uint8_t> reset_counter_;
  bool status_healthy_{false};
  std::unique_ptr<const uav_ommpc_controller::reference::PolynomialReference> trajectory_;
  std::int64_t reference_start_ns_{}, last_ros_ns_{}, odometry_ros_ns_{};
  ThrustRates output_;
  rclcpp::Publisher<ThrustRates>::SharedPtr output_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<PolynomialTrajectory>::SharedPtr reference_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<OmmpcNode>());  // Single executor thread owns all cached inputs.
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ommpc_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
