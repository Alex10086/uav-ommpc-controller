#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "ommpc_inputs.hpp"

using uav_ommpc_controller::dynamics::State;
using uav_ommpc_interfaces::msg::ReferenceState;
using uav_ommpc_ros::decode_odometry;
using uav_ommpc_ros::decode_reference;
using uav_ommpc_ros::reference_index;
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

ReferenceState reference()
{
  ReferenceState message;
  message.position = {1.0000000001, -2.0000000002, 3.0000000003};
  message.velocity = {-4.0000000004, 5.0000000005, -6.0000000006};
  message.q = {1.0, 0.0, 0.0, 0.0};
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

TEST(OmmpcReference, PreservesDoublePrecisionAndHamiltonRotation)
{
  auto message = reference();
  const double angle = 0.123456789012345;
  message.q = {std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)};
  State out;
  ASSERT_TRUE(decode_reference(message, out));
  for (std::size_t axis = 0; axis < 3; ++axis) {
    EXPECT_DOUBLE_EQ(out.p[axis], message.position[axis]);
    EXPECT_DOUBLE_EQ(out.v[axis], message.velocity[axis]);
  }
  EXPECT_NEAR(out.R(0, 0), std::cos(angle), 1e-15);
  EXPECT_NEAR(out.R(1, 0), std::sin(angle), 1e-15);
  EXPECT_NEAR(out.R(0, 1), -std::sin(angle), 1e-15);
}

TEST(OmmpcReference, NormalizesWithinDoubleTolerance)
{
  for (const double scale : {0.9999995, 1.0000005, -1.0000005}) {
    auto message = reference();
    const double component = scale * std::sqrt(0.5);
    message.q = {component, 0.0, component, 0.0};
    State out;
    ASSERT_TRUE(decode_reference(message, out));
    EXPECT_TRUE((out.R * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitX(), 1e-14));
    EXPECT_NEAR(out.R.determinant(), 1.0, 1e-14);
  }
}

TEST(OmmpcReference, RejectsInvalidQuaternionsAndPreservesOutput)
{
  for (const double scalar : {0.0, 0.999998, 1.000002, std::numeric_limits<double>::max()}) {
    auto message = reference();
    message.q = {scalar, 0.0, 0.0, 0.0};
    expect_rejected(message, decode_reference);
  }
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    for (std::size_t axis = 0; axis < 4; ++axis) {
      auto message = reference();
      message.q[axis] = invalid;
      expect_rejected(message, decode_reference);
    }
  }
}

TEST(OmmpcReference, RejectsNonfinitePositionVelocityAndPreservesOutput)
{
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      auto message = reference();
      message.position[axis] = invalid;
      expect_rejected(message, decode_reference);
      message = reference();
      message.velocity[axis] = invalid;
      expect_rejected(message, decode_reference);
    }
  }
}

TEST(OmmpcReferenceIndex, FloorsExactNanosecondBoundaries)
{
  constexpr std::int64_t start = 123456789;
  constexpr std::int64_t dt = 20000000;
  EXPECT_EQ(reference_index(start, start, dt, 30, 20), 0u);
  EXPECT_EQ(reference_index(start + dt - 1, start, dt, 30, 20), 0u);
  EXPECT_EQ(reference_index(start + dt, start, dt, 30, 20), 1u);
  EXPECT_EQ(reference_index(start + dt + 1, start, dt, 30, 20), 1u);
  EXPECT_EQ(reference_index(start + 11 * dt - 1, start, dt, 30, 20), 10u);
  EXPECT_FALSE(reference_index(start + 11 * dt, start, dt, 30, 20));
}

TEST(OmmpcReferenceIndex, RequiresWholeHorizonWithoutTerminalHold)
{
  EXPECT_EQ(reference_index(100, 100, 20, 20, 20), 0u);
  EXPECT_EQ(reference_index(119, 100, 20, 20, 20), 0u);
  EXPECT_FALSE(reference_index(120, 100, 20, 20, 20));
  EXPECT_FALSE(reference_index(100, 100, 20, 19, 20));
  EXPECT_FALSE(reference_index(100, 100, 20, 0, 20));
  EXPECT_FALSE(reference_index(100, 100, 20, 20, 0));
  EXPECT_FALSE(reference_index(100, 100, 20, 0, 0));
}

TEST(OmmpcReferenceIndex, RejectsFutureAndNonpositiveTimes)
{
  EXPECT_FALSE(reference_index(99, 100, 20, 30, 20));
  for (const std::int64_t invalid : {std::int64_t{0}, std::int64_t{-1},
      std::numeric_limits<std::int64_t>::min()}) {
    EXPECT_FALSE(reference_index(invalid, 100, 20, 30, 20));
    EXPECT_FALSE(reference_index(100, invalid, 20, 30, 20));
    EXPECT_FALSE(reference_index(100, 100, invalid, 30, 20));
  }
}

TEST(OmmpcReferenceIndex, AvoidsOverflowAtLargeTimestampsAndHorizons)
{
  constexpr auto maximum = std::numeric_limits<std::int64_t>::max();
  EXPECT_EQ(reference_index(maximum, maximum - 100, 10, 30, 20), 10u);
  EXPECT_EQ(reference_index(maximum, 1, maximum, 20, 20), 0u);
  EXPECT_EQ(reference_index(maximum, 1, maximum - 1, 21, 20), 1u);
  EXPECT_FALSE(reference_index(maximum, 1, 1, 30, 20));
  const auto capacity = std::numeric_limits<std::size_t>::max();
  EXPECT_FALSE(reference_index(maximum, 1, 1, capacity, capacity));
  EXPECT_EQ(reference_index(2, 1, 1, capacity, capacity - 1), 1u);
}
