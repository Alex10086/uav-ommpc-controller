#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "ommpc_inputs.hpp"

using uav_ommpc_controller::dynamics::State;
using uav_ommpc_interfaces::msg::PolynomialTrajectory;
using uav_ommpc_ros::decode_odometry;
using uav_ommpc_ros::decode_polynomial;
using uav_ommpc_ros::reference_elapsed;
using Odometry = px4_msgs::msg::VehicleOdometry;

namespace
{
Odometry odometry()
{
  Odometry message;
  message.pose_frame = Odometry::POSE_FRAME_NED;
  message.velocity_frame = Odometry::VELOCITY_FRAME_NED;
  message.position = {1.0f, -2.0f, 3.0f};
  message.velocity = {-4.0f, 5.0f, -6.0f};
  message.q = {1.0f, 0.0f, 0.0f, 0.0f};
  return message;
}

PolynomialTrajectory reference()
{
  PolynomialTrajectory message;
  message.header.frame_id = "ned";
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 345;
  message.segments.resize(1);
  auto &segment = message.segments.front();
  segment.duration = 2.0;
  segment.x = {1.0000000001, 0.25};
  segment.y = {-2.0000000002};
  segment.z = {-3.0000000003};
  segment.yaw = {0.123456789012345};
  return message;
}

State sentinel()
{
  State state;
  state.p = Eigen::Vector3d(11.0, 12.0, 13.0);
  state.v = Eigen::Vector3d(21.0, 22.0, 23.0);
  state.R = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return state;
}

void expect_unchanged(const State &actual, const State &expected)
{
  EXPECT_TRUE((actual.p.array() == expected.p.array()).all());
  EXPECT_TRUE((actual.v.array() == expected.v.array()).all());
  EXPECT_TRUE((actual.R.array() == expected.R.array()).all());
}

template<typename Message, typename Decoder>
void expect_rejected(const Message &message, Decoder decode)
{
  const State before = sentinel();
  State out = before;
  EXPECT_FALSE(decode(message, out));
  expect_unchanged(out, before);
}
}  // namespace

TEST(OmmpcOdometry, DecodesNedAndIgnoresUnusedFields)
{
  auto message = odometry();
  message.quality = 0;
  message.angular_velocity.fill(std::numeric_limits<float>::quiet_NaN());
  message.position_variance.fill(std::numeric_limits<float>::quiet_NaN());
  message.orientation_variance.fill(std::numeric_limits<float>::quiet_NaN());
  message.velocity_variance.fill(std::numeric_limits<float>::quiet_NaN());
  State out;
  ASSERT_TRUE(decode_odometry(message, out));
  EXPECT_TRUE(out.p.isApprox(Eigen::Vector3d(1.0, -2.0, 3.0), 1e-14));
  EXPECT_TRUE(out.v.isApprox(Eigen::Vector3d(-4.0, 5.0, -6.0), 1e-14));
  EXPECT_TRUE(out.R.isApprox(Eigen::Matrix3d::Identity(), 1e-14));
}

TEST(OmmpcOdometry, HamiltonQuaternionRotatesBodyToWorldWithoutRotatingNedVelocity)
{
  auto message = odometry();
  const float half = static_cast<float>(std::sqrt(0.5));
  message.q = {half, 0.0f, 0.0f, half};
  State out;
  ASSERT_TRUE(decode_odometry(message, out));
  EXPECT_TRUE((out.R * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-14));
  EXPECT_TRUE((out.R * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitZ(), 1e-14));
  EXPECT_TRUE(out.v.isApprox(Eigen::Vector3d(-4.0, 5.0, -6.0), 1e-14));

  message.velocity_frame = Odometry::VELOCITY_FRAME_BODY_FRD;
  ASSERT_TRUE(decode_odometry(message, out));
  EXPECT_TRUE(out.v.isApprox(Eigen::Vector3d(-5.0, -4.0, -6.0), 1e-14));
  EXPECT_TRUE(out.p.isApprox(Eigen::Vector3d(1.0, -2.0, 3.0), 1e-14));
}

TEST(OmmpcOdometry, NormalizesFloatQuaternionWithinTolerance)
{
  for (const double scale : {0.9995, 1.0005, -1.0005}) {
    auto message = odometry();
    const float component = static_cast<float>(scale * std::sqrt(0.5));
    message.q = {component, component, 0.0f, 0.0f};
    State out;
    ASSERT_TRUE(decode_odometry(message, out));
    EXPECT_TRUE((out.R * Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitZ(), 1e-14));
    EXPECT_TRUE((out.R.transpose() * out.R).isApprox(Eigen::Matrix3d::Identity(), 1e-14));
    EXPECT_NEAR(out.R.determinant(), 1.0, 1e-14);
  }
}

TEST(OmmpcOdometry, RejectsInvalidFramesAndPreservesOutput)
{
  for (const auto frame : {Odometry::POSE_FRAME_UNKNOWN, Odometry::POSE_FRAME_FRD}) {
    auto message = odometry();
    message.pose_frame = frame;
    expect_rejected(message, decode_odometry);
  }
  for (const auto frame : {Odometry::VELOCITY_FRAME_UNKNOWN, Odometry::VELOCITY_FRAME_FRD}) {
    auto message = odometry();
    message.velocity_frame = frame;
    expect_rejected(message, decode_odometry);
  }
}

TEST(OmmpcOdometry, RejectsNonfinitePositionVelocityAndPreservesOutput)
{
  for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      auto message = odometry();
      message.position[axis] = invalid;
      expect_rejected(message, decode_odometry);
      message = odometry();
      message.velocity[axis] = invalid;
      expect_rejected(message, decode_odometry);
    }
  }
}

TEST(OmmpcOdometry, RejectsZeroNonunitAndNonfiniteQuaternions)
{
  for (const float scalar : {0.0f, 0.998f, 1.002f, std::numeric_limits<float>::max()}) {
    auto message = odometry();
    message.q = {scalar, 0.0f, 0.0f, 0.0f};
    expect_rejected(message, decode_odometry);
  }
  for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()}) {
    for (std::size_t axis = 0; axis < 4; ++axis) {
      auto message = odometry();
      message.q[axis] = invalid;
      expect_rejected(message, decode_odometry);
    }
  }
}

TEST(OmmpcPolynomial, DecodesNormalizedCoefficientsAndTimestamp)
{
  const auto message = reference();
  std::int64_t start = 0;
  const auto plan = decode_polynomial(message, start, 9.81);
  EXPECT_EQ(start, 12000000345LL);
  EXPECT_DOUBLE_EQ(plan->duration(), 2.0);
  uav_ommpc_controller::reference::FlatSample sample;
  ASSERT_TRUE(plan->sample_flat(1.0, sample));
  EXPECT_DOUBLE_EQ(sample.p.x(), message.segments[0].x[0] + 0.125);
  EXPECT_DOUBLE_EQ(sample.v.x(), 0.125);
  EXPECT_DOUBLE_EQ(sample.p.y(), message.segments[0].y[0]);
  EXPECT_DOUBLE_EQ(sample.p.z(), message.segments[0].z[0]);
  EXPECT_DOUBLE_EQ(sample.yaw, message.segments[0].yaw[0]);
}

TEST(OmmpcPolynomial, RejectsMetadataWithoutChangingStart)
{
  const auto reject = [](const PolynomialTrajectory &message) {
      std::int64_t start = 77;
      EXPECT_THROW(decode_polynomial(message, start, 9.81), std::invalid_argument);
      EXPECT_EQ(start, 77);
    };
  auto message = reference();
  message.header.frame_id = "enu";
  reject(message);
  message = reference();
  message.header.stamp.sec = -1;
  reject(message);
  message = reference();
  message.header.stamp.nanosec = 1000000000u;
  reject(message);
  message = reference();
  message.header.stamp.sec = 0;
  message.header.stamp.nanosec = 0;
  reject(message);
  message = reference();
  message.segments.clear();
  reject(message);
}

TEST(OmmpcPolynomial, AppliesConstructorValidationAtReceipt)
{
  std::int64_t start = 77;
  auto message = reference();
  message.segments[0].x.clear();
  EXPECT_THROW(decode_polynomial(message, start, 9.81), std::invalid_argument);
  message = reference();
  message.segments[0].duration = 0.0;
  EXPECT_THROW(decode_polynomial(message, start, 9.81), std::invalid_argument);
  message = reference();
  message.segments[0].yaw[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(decode_polynomial(message, start, 9.81), std::invalid_argument);
  message = reference();
  message.segments.push_back(message.segments[0]);  // Position jumps back at the knot.
  EXPECT_THROW(decode_polynomial(message, start, 9.81), std::invalid_argument);
  message = reference();
  message.hold_final = true;  // Nonzero terminal velocity cannot become a held hover.
  EXPECT_THROW(decode_polynomial(message, start, 9.81), std::invalid_argument);
  EXPECT_EQ(start, 77);
}

TEST(OmmpcPolynomial, DistinguishesFiniteGeometryAndExplicitFinalHold)
{
  auto message = reference();
  std::int64_t start = 0;
  auto plan = decode_polynomial(message, start, 9.81);
  State state;
  uav_ommpc_controller::dynamics::ControlInput input;
  EXPECT_TRUE(plan->sample(2.0, state, input));
  EXPECT_FALSE(plan->sample(2.000001, state, input));
  message.segments[0].x = {1.0};
  message.hold_final = true;
  plan = decode_polynomial(message, start, 9.81);
  ASSERT_TRUE(plan->sample(1000.0, state, input));
  EXPECT_DOUBLE_EQ(state.p.x(), 1.0);
  EXPECT_TRUE(state.v.isZero());
  EXPECT_NEAR(input.thrust_acceleration, 9.81, 1e-12);
}

TEST(OmmpcPolynomialTime, UsesContinuousReceiptAnchorWithoutFlooring)
{
  constexpr std::int64_t start = 12000000345LL;
  const auto elapsed = reference_elapsed(start + 19999999, start);
  ASSERT_TRUE(elapsed);
  EXPECT_NEAR(*elapsed, 0.019999999, 1e-16);
  EXPECT_NEAR(*reference_elapsed(start + 20000001, start), 0.020000001, 1e-16);
  auto message = reference();
  std::int64_t decoded_start = 0;
  const auto plan = decode_polynomial(message, decoded_start, 9.81);
  std::vector<State> states(3);
  std::vector<uav_ommpc_controller::dynamics::ControlInput> inputs(2);
  ASSERT_TRUE(plan->sample_horizon(*elapsed, 0.02, states, inputs));
  EXPECT_NEAR(states[0].p.x(), message.segments[0].x[0] + *elapsed * 0.125, 1e-14);
  EXPECT_NEAR(states[2].p.x(), states[0].p.x() + 0.04 * 0.125, 1e-14);
  EXPECT_FALSE(plan->sample_horizon(1.98, 0.02, states, inputs));
}

TEST(OmmpcPolynomialTime, RejectsFutureAndInvalidTimesWithoutOverflow)
{
  EXPECT_FALSE(reference_elapsed(99, 100));
  EXPECT_FALSE(reference_elapsed(100, 0));
  EXPECT_FALSE(reference_elapsed(100, -1));
  EXPECT_FALSE(reference_elapsed(std::numeric_limits<std::int64_t>::min(), 1));
  ASSERT_TRUE(reference_elapsed(100, 100));
  EXPECT_DOUBLE_EQ(*reference_elapsed(100, 100), 0.0);
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  ASSERT_TRUE(reference_elapsed(maximum, maximum - 100));
  EXPECT_NEAR(*reference_elapsed(maximum, maximum - 100), 1e-7, 1e-22);
  EXPECT_TRUE(std::isfinite(*reference_elapsed(maximum, 1)));
}
