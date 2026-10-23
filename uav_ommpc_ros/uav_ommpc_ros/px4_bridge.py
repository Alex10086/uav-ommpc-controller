"""Relay PX4 telemetry and stream externally supplied body-rate/thrust commands."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
)
from uav_ommpc_interfaces.msg import ThrustRates


class Px4Bridge(Node):
    def __init__(self):
        super().__init__('px4_bridge')
        self.hover_thrust = self.declare_parameter('hover_thrust', 0.60).value
        rate = self.declare_parameter('rate', 50.0).value
        self.command_timeout = self.declare_parameter('command_timeout', 0.5).value
        self.target_system = self.declare_parameter('target_system', 1).value
        self.target_component = self.declare_parameter('target_component', 1).value
        if not math.isfinite(self.hover_thrust) or not 0.0 < self.hover_thrust <= 1.0:
            raise ValueError('hover_thrust must be finite and in (0, 1]')
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('rate must be finite and positive')
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0.0:
            raise ValueError('command_timeout must be finite and positive')
        if not 1 <= self.target_system <= 255 or not 1 <= self.target_component <= 255:
            raise ValueError('target_system and target_component must be in [1, 255]')

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        telemetry = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_pub = self.create_publisher(VehicleOdometry, '/uav/state', telemetry)
        self.status_pub = self.create_publisher(VehicleStatus, '/uav/status', telemetry)
        self.ack_pub = self.create_publisher(VehicleCommandAck, '/uav/command_ack', telemetry)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', reliable)
        self.rates_pub = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', reliable)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', reliable)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.state_pub.publish, telemetry)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_pub.publish, telemetry)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_pub.publish, telemetry)
        self.create_subscription(ThrustRates, '/uav/control_input', self.on_control, reliable)
        self.create_subscription(
            VehicleCommand, '/uav/vehicle_command', self.on_vehicle_command, reliable)
        self.control = None
        self.last_control_time = 0.0
        self.create_timer(1.0 / rate, self.publish_control)

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def on_control(self, message):
        if (not math.isfinite(message.thrust_acceleration)
                or message.thrust_acceleration < 0.0
                or not all(math.isfinite(value) for value in message.body_rates)):
            self.get_logger().warning('Rejected nonfinite rates/thrust or negative thrust')
            return
        self.control = message
        self.last_control_time = time.monotonic()

    def on_vehicle_command(self, message):
        message.timestamp = self.timestamp_us()
        message.source_system = 250
        message.source_component = 190
        message.target_system = self.target_system
        message.target_component = self.target_component
        message.from_external = True
        self.command_pub.publish(message)

    def publish_control(self):
        if (self.control is None
                or time.monotonic() - self.last_control_time > self.command_timeout):
            return
        timestamp = self.timestamp_us()
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.body_rate = True
        rates = VehicleRatesSetpoint()
        rates.timestamp = timestamp
        rates.roll, rates.pitch, rates.yaw = self.control.body_rates
        throttle = min(max(self.hover_thrust * self.control.thrust_acceleration / 9.80665, 0.0), 1.0)
        rates.thrust_body = [0.0, 0.0, -throttle]
        self.mode_pub.publish(mode)
        self.rates_pub.publish(rates)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Px4Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
