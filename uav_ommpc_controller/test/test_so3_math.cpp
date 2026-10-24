#include <array>
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "uav_ommpc_controller/so3_math.hpp"

namespace
{
namespace so3 = uav_ommpc_controller::so3;
namespace r3 = uav_ommpc_controller::r3;
using Eigen::Matrix3d;
using Eigen::Vector3d;
constexpr double kPi = 3.14159265358979323846;

void expect_rotation(const Matrix3d &rotation)
{
  EXPECT_LT((rotation.transpose() * rotation - Matrix3d::Identity()).norm(), 3e-14);
  EXPECT_NEAR(rotation.determinant(), 1.0, 3e-14);
}

TEST(So3Math, HatVeeAndCrossProduct)
{
  const Vector3d v(0.3, -0.7, 1.2);
  const Vector3d w(-0.8, 0.1, 0.4);
  EXPECT_LT((so3::hat(v) * w - v.cross(w)).norm(), 1e-15);
  EXPECT_EQ((so3::hat(v) + so3::hat(v).transpose()).norm(), 0.0);
  EXPECT_EQ((so3::vee(so3::hat(v)) - v).norm(), 0.0);
}

TEST(So3Math, EuclideanErrorIsActualMinusReference)
{
  const Vector3d reference(2.0, -3.0, 1.0);
  const Vector3d actual(2.5, -4.0, 3.0);
  const Vector3d delta(0.5, -1.0, 2.0);
  EXPECT_EQ((r3::boxminus(actual, reference) - delta).norm(), 0.0);
  EXPECT_EQ((r3::boxplus(reference, delta) - actual).norm(), 0.0);
}

TEST(So3Math, IdentityAndZeroLimits)
{
  const Vector3d zero = Vector3d::Zero();
  const Matrix3d identity = Matrix3d::Identity();
  EXPECT_EQ((so3::exp(zero) - identity).norm(), 0.0);
  EXPECT_EQ(so3::log(identity).norm(), 0.0);
  EXPECT_EQ((so3::left_jacobian(zero) - identity).norm(), 0.0);
  EXPECT_EQ((so3::right_jacobian(zero) - identity).norm(), 0.0);
  EXPECT_EQ((so3::right_jacobian_inverse(zero) - identity).norm(), 0.0);
}

TEST(So3Math, PositiveQuarterTurnsAreRightHanded)
{
  const std::array<Vector3d, 3> axes{Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
  for (int i = 0; i < 3; ++i)
  {
    const Matrix3d rotation = so3::exp(0.5 * kPi * axes[i]);
    EXPECT_LT((rotation * axes[(i + 1) % 3] - axes[(i + 2) % 3]).norm(), 3e-16);
    EXPECT_LT((so3::log(rotation) - 0.5 * kPi * axes[i]).norm(), 3e-16);
  }
}

TEST(So3Math, TinyRotationsAreNotDiscarded)
{
  const Vector3d axis = Vector3d(1.0, -2.0, 3.0).normalized();
  for (double angle : {1e-14, 1e-10, 1e-7, 1e-4})
  {
    for (double sign : {-1.0, 1.0})
    {
      const Vector3d phi = sign * angle * axis;
      const Matrix3d rotation = so3::exp(phi);
      EXPECT_LT((so3::log(rotation) - phi).norm(), angle * 2e-15);
      EXPECT_LT((rotation - Eigen::AngleAxisd(sign * angle, axis).toRotationMatrix()).norm(), 3e-16);
      expect_rotation(rotation);
    }
  }
}

TEST(So3Math, ExpAgreesWithAxisAngleAcrossSeriesBoundary)
{
  const Vector3d axis = Vector3d(-2.0, 3.0, 1.0).normalized();
  for (double angle : {0.000999999, 0.001, 0.001000001, 0.1, 1.3, kPi, 2.0 * kPi, 7.0})
  {
    const Matrix3d rotation = so3::exp(angle * axis);
    EXPECT_LT((rotation - Eigen::AngleAxisd(angle, axis).toRotationMatrix()).norm(), 4e-15);
    expect_rotation(rotation);
  }
}

TEST(So3Math, LogHandlesBothSidesOfPiAndMixedSignAxes)
{
  const std::array<Vector3d, 5> axes{
    Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
    Vector3d(1.0, -2.0, 3.0).normalized(), Vector3d(-3.0, 1.0, -2.0).normalized()};
  for (const Vector3d &axis : axes)
  {
    for (double offset : {-1e-6, -1e-10, 0.0, 1e-10, 1e-6})
    {
      const Matrix3d rotation = Eigen::AngleAxisd(kPi + offset, axis).toRotationMatrix();
      const Vector3d phi = so3::log(rotation);
      ASSERT_TRUE(phi.allFinite());
      EXPECT_LE(phi.norm(), kPi + 2e-15);
      EXPECT_LT((so3::exp(phi) - rotation).norm(), 3e-15);
      if (offset != 0.0)
      {
        const Vector3d expected = (offset < 0.0 ? kPi + offset : -kPi + offset) * axis;
        EXPECT_LT((phi - expected).norm(), 2e-15);
      }
    }
  }
}

TEST(So3Math, SeededRoundTripsPreserveSO3AndPrincipalBranch)
{
  std::mt19937 generator(42);
  std::uniform_real_distribution<double> component(-1.0, 1.0);
  std::uniform_real_distribution<double> angle(-2.0 * kPi, 2.0 * kPi);
  for (int sample = 0; sample < 500; ++sample)
  {
    Vector3d axis;
    for (int i = 0; i < 3; ++i)
    {
      axis[i] = component(generator);
    }
    axis.normalize();
    const double theta = angle(generator);
    const Vector3d input = theta * axis;
    const Matrix3d rotation = so3::exp(input);
    const Vector3d principal = so3::log(rotation);
    expect_rotation(rotation);
    EXPECT_LE(principal.norm(), kPi + 2e-15);
    EXPECT_LT((so3::exp(principal) - rotation).norm(), 4e-15);
    EXPECT_LT((rotation - Eigen::AngleAxisd(theta, axis).toRotationMatrix()).norm(), 4e-15);
    if (std::abs(theta) < kPi)
    {
      EXPECT_LT((principal - input).norm(), 2e-15);
    }
  }
}

TEST(So3Math, LocalErrorUsesRightPerturbationAndActualFirst)
{
  const Matrix3d reference = Eigen::AngleAxisd(0.7, Vector3d::UnitX()).toRotationMatrix();
  const Vector3d delta(0.0, 0.2, 0.0);
  const Matrix3d actual = reference * Eigen::AngleAxisd(0.2, Vector3d::UnitY()).toRotationMatrix();
  EXPECT_LT((so3::boxplus(reference, delta) - actual).norm(), 3e-16);
  EXPECT_LT((so3::boxminus(actual, reference) - delta).norm(), 3e-16);
  EXPECT_LT((so3::boxminus(reference, actual) + delta).norm(), 3e-16);
  EXPECT_LT((so3::boxplus(reference, so3::boxminus(actual, reference)) - actual).norm(), 3e-16);
  const Matrix3d world_change = so3::exp(Vector3d(-0.4, 0.1, 0.3));
  EXPECT_LT((so3::boxminus(world_change * actual, world_change * reference) - delta).norm(), 5e-16);
}

TEST(So3Math, JacobiansMatchLeftAndRightFiniteDifferences)
{
  const Vector3d axis = Vector3d(1.0, -2.0, 3.0).normalized();
  constexpr double h = 1e-6;
  for (double theta : {0.0, 1e-10, 0.000999999, 0.001, 0.001000001, 0.7, kPi - 1e-8})
  {
    const Vector3d phi = theta * axis;
    const Matrix3d rotation = so3::exp(phi);
    Matrix3d left_difference;
    Matrix3d right_difference;
    for (int i = 0; i < 3; ++i)
    {
      const Matrix3d plus = so3::exp(phi + h * Vector3d::Unit(i));
      const Matrix3d minus = so3::exp(phi - h * Vector3d::Unit(i));
      left_difference.col(i) =
        (so3::log(plus * rotation.transpose()) - so3::log(minus * rotation.transpose())) / (2.0 * h);
      right_difference.col(i) =
        (so3::boxminus(plus, rotation) - so3::boxminus(minus, rotation)) / (2.0 * h);
    }
    EXPECT_LT((so3::left_jacobian(phi) - left_difference).norm(), 1e-9);
    EXPECT_LT((so3::right_jacobian(phi) - right_difference).norm(), 1e-9);
  }
}

TEST(So3Math, RightJacobianInverseMatchesLogDerivativeAndMatrixInverse)
{
  const Vector3d axis = Vector3d(1.0, -2.0, 3.0).normalized();
  constexpr double h = 1e-6;
  for (double theta : {0.0, 1e-10, 0.000999999, 0.001, 0.001000001, 0.7, kPi - 0.002})
  {
    const Vector3d phi = theta * axis;
    const Matrix3d inverse = so3::right_jacobian_inverse(phi);
    EXPECT_LT((inverse * so3::right_jacobian(phi) - Matrix3d::Identity()).norm(), 2e-14);
    Matrix3d numeric;
    for (int column = 0; column < 3; ++column)
    {
      numeric.col(column) =
        (so3::log(so3::exp(phi) * so3::exp(h * Vector3d::Unit(column))) -
        so3::log(so3::exp(phi) * so3::exp(-h * Vector3d::Unit(column)))) / (2.0 * h);
    }
    EXPECT_LT((inverse - numeric).norm(), 2e-9);
  }
}

TEST(So3Math, TinyJacobianTermsRetainTheirSigns)
{
  const Vector3d phi(1e-10, -2e-10, 3e-10);
  const Matrix3d left = so3::left_jacobian(phi);
  const Matrix3d right = so3::right_jacobian(phi);
  EXPECT_LT((so3::vee(left - left.transpose()) - phi).norm(), 1e-24);
  EXPECT_LT((so3::vee(right - right.transpose()) + phi).norm(), 1e-24);
}

TEST(So3Math, LocalVectorActionHasExpectedDerivative)
{
  const Matrix3d rotation = so3::exp(Vector3d(0.2, -0.4, 0.7));
  const Vector3d point(1.0, -2.0, 0.3);
  constexpr double h = 1e-6;
  Matrix3d derivative;
  for (int i = 0; i < 3; ++i)
  {
    derivative.col(i) = (so3::boxplus(rotation, h * Vector3d::Unit(i)) * point -
      so3::boxplus(rotation, -h * Vector3d::Unit(i)) * point) / (2.0 * h);
  }
  EXPECT_LT((derivative + rotation * so3::hat(point)).norm(), 1e-9);
}

TEST(So3Math, RepeatedBodyRateUpdatesTrackClosedFormRotation)
{
  const Matrix3d initial = so3::exp(Vector3d(0.4, 0.2, -0.1));
  const Vector3d omega(0.1, -0.2, 0.3);
  constexpr double dt = 0.001;
  Matrix3d rotation = initial;
  for (int step = 0; step < 10000; ++step)
  {
    rotation = so3::boxplus(rotation, omega * dt);
  }
  const Matrix3d expected = initial * Eigen::AngleAxisd(10.0 * omega.norm(), omega.normalized()).toRotationMatrix();
  EXPECT_LT((rotation - expected).norm(), 2e-12);
  EXPECT_LT((rotation.transpose() * rotation - Matrix3d::Identity()).norm(), 3e-12);
}
}  // namespace
