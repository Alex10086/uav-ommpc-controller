#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <uav_ommpc_interfaces/msg/thrust_rates.hpp>

#include "command_state.hpp"

namespace
{
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleRatesSetpoint;
using px4_msgs::msg::VehicleStatus;
using uav_ommpc_interfaces::msg::ThrustRates;
using uav_ommpc_ros::CommandState;
using uav_ommpc_ros::Limits;

class Px4Bridge final : public rclcpp::Node
{
public:
  Px4Bridge() : Node("px4_bridge")
  {
    const auto hover_thrust = declare_parameter<double>("hover_thrust", 0.60);
    const auto max_body_rates =
      declare_parameter<std::vector<double>>("max_body_rates", std::vector<double>{3.0, 3.0, 1.5});
    const auto max_throttle = declare_parameter<double>("max_throttle", 0.9);
    const auto rate = declare_parameter<double>("rate", 50.0);
    const auto timeout = declare_parameter<double>("command_timeout", 0.5);
    const auto target_system = declare_parameter<std::int64_t>("target_system", 1);
    const auto target_component = declare_parameter<std::int64_t>("target_component", 1);
    if (!std::isfinite(hover_thrust) || hover_thrust <= 0.0 || hover_thrust > 1.0) {
      throw std::invalid_argument("hover_thrust must be finite and in (0, 1]");
    }
    if (max_body_rates.size() != 3 ||
      !std::all_of(max_body_rates.begin(), max_body_rates.end(),
        [](double limit) {return std::isfinite(limit) && limit > 0.0;}))
    {
      throw std::invalid_argument("max_body_rates must contain three finite positive values");
    }
    if (!std::isfinite(max_throttle) || max_throttle <= 0.0 || max_throttle > 1.0) {
      throw std::invalid_argument("max_throttle must be finite and in (0, 1]");
    }
    if (!std::isfinite(rate) || rate <= 0.0) {
      throw std::invalid_argument("rate must be finite and positive");
    }
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      throw std::invalid_argument("command_timeout must be finite and positive");
    }
    if (target_system < 1 || target_system > 255 ||
      target_component < 1 || target_component > 255)
    {
      throw std::invalid_argument("target_system and target_component must be in [1, 255]");
    }
    target_system_ = static_cast<std::uint8_t>(target_system);
    target_component_ = static_cast<std::uint8_t>(target_component);
    state_.emplace(
      Limits{hover_thrust, {max_body_rates[0], max_body_rates[1], max_body_rates[2]}, max_throttle},
      std::chrono::duration_cast<CommandState::Clock::duration>(
        std::chrono::duration<double>(timeout)));

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto telemetry = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_pub_ = create_publisher<VehicleOdometry>("/uav/state", telemetry);
    status_pub_ = create_publisher<VehicleStatus>("/uav/status", telemetry);
    ack_pub_ = create_publisher<VehicleCommandAck>("/uav/command_ack", telemetry);
    mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", reliable);
    rates_pub_ = create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", reliable);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", reliable);
    state_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", telemetry,
      [this](const VehicleOdometry & message) {state_pub_->publish(message);});
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", telemetry,
      [this](const VehicleStatus & message) {
        offboard_ = message.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        status_pub_->publish(message);
      });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", telemetry,
      [this](const VehicleCommandAck & message) {ack_pub_->publish(message);});
    control_sub_ = create_subscription<ThrustRates>(
      "/uav/control_input", reliable,
      [this](const ThrustRates & message) {on_control(message);});
    command_sub_ = create_subscription<VehicleCommand>(
      "/uav/vehicle_command", reliable,
      [this](VehicleCommand::SharedPtr message) {
        message->timestamp = timestamp_us();
        message->source_system = 250;
        message->source_component = 190;
        message->target_system = target_system_;
        message->target_component = target_component_;
        message->from_external = true;
        command_pub_->publish(*message);
      });
    mode_.body_rate = true;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {publish_control();});
  }

private:
  std::uint64_t timestamp_us()
  {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void on_control(const ThrustRates & message)
  {
    if (!state_->accept(message, CommandState::Clock::now())) {
      RCLCPP_WARN(get_logger(), "Rejected nonfinite rates/thrust or negative thrust");
      return;
    }
    if (state_->command().limited) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Control input limited by max_body_rates/max_throttle");
    }
  }

  void publish_control()
  {
    if (!state_->fresh(CommandState::Clock::now())) {
      return;
    }
    const auto timestamp = timestamp_us();
    mode_.timestamp = timestamp;
    mode_pub_->publish(mode_);
    // PX4 rate control uses the newest rate setpoint from any source, so never
    // compete with PX4's own attitude controller outside Offboard.
    if (!offboard_) {
      return;
    }
    const auto & command = state_->command();
    rates_.timestamp = timestamp;
    rates_.roll = command.body_rates[0];
    rates_.pitch = command.body_rates[1];
    rates_.yaw = command.body_rates[2];
    rates_.thrust_body[2] = -command.throttle;
    rates_pub_->publish(rates_);
  }

  std::uint8_t target_system_{};
  std::uint8_t target_component_{};
  std::optional<CommandState> state_;
  bool offboard_{false};
  OffboardControlMode mode_{};
  VehicleRatesSetpoint rates_{};
  rclcpp::Publisher<VehicleOdometry>::SharedPtr state_pub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<VehicleCommandAck>::SharedPtr ack_pub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr state_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<ThrustRates>::SharedPtr control_sub_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Px4Bridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("px4_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
