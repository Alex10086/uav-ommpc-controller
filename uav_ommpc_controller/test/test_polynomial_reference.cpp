#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "uav_ommpc_controller/polynomial_reference.hpp"

namespace
{
namespace reference = uav_ommpc_controller::reference;
namespace dynamics = uav_ommpc_controller::dynamics;
using reference::FlatSample;
using reference::PolynomialReference;
using reference::Segment;
using Eigen::Vector3d;

Segment hover(double duration = 2.0)
{
  return {duration, {1.0}, {-2.0}, {-3.0}, {0.4}};
}

void expect_flat_near(const FlatSample &a, const FlatSample &b, double tolerance)
{
  EXPECT_LE((a.p - b.p).norm(), tolerance);
  EXPECT_LE((a.v - b.v).norm(), tolerance);
  EXPECT_LE((a.a - b.a).norm(), tolerance);
  EXPECT_LE((a.j - b.j).norm(), tolerance);
  EXPECT_NEAR(a.yaw, b.yaw, tolerance);
  EXPECT_NEAR(a.yaw_rate, b.yaw_rate, tolerance);
}

TEST(PolynomialReference, HornerDerivativesUsePhysicalTimeAndIndependentDegrees)
{
  for (double duration : {0.25, 1.0, 4.0})
  {
    const PolynomialReference ref({{duration, {1, 2, 3, 4}, {-2, 5}, {-3}, {0.4, 0.6, 0.2}}});
    for (double s : {0.0, 0.3, 1.0})
    {
      FlatSample flat;
      ASSERT_TRUE(ref.sample_flat(s * duration, flat));
      const FlatSample expected{
        {1 + 2*s + 3*s*s + 4*s*s*s, -2 + 5*s, -3},
        {(2 + 6*s + 12*s*s)/duration, 5/duration, 0},
        {(6 + 24*s)/(duration*duration), 0, 0},
        {24/(duration*duration*duration), 0, 0},
        0.4 + 0.6*s + 0.2*s*s, (0.6 + 0.4*s)/duration};
      expect_flat_near(flat, expected, 1e-11);
    }
  }
}

TEST(PolynomialReference, NinthDegreeAndTinyConstantDuration)
{
  Segment ninth = hover(2);
  ninth.x = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  const PolynomialReference ref({ninth});
  FlatSample f;
  ASSERT_TRUE(ref.sample_flat(1, f));
  EXPECT_DOUBLE_EQ(f.p.x(), std::pow(0.5, 9));
  EXPECT_DOUBLE_EQ(f.v.x(), 9*std::pow(0.5, 8)/2);
  EXPECT_DOUBLE_EQ(f.a.x(), 72*std::pow(0.5, 7)/4);
  EXPECT_DOUBLE_EQ(f.j.x(), 504*std::pow(0.5, 6)/8);
  const PolynomialReference tiny({hover(1e-200)});
  ASSERT_TRUE(tiny.sample_flat(0, f));
  EXPECT_EQ(f.j.norm(), 0);
}

TEST(PolynomialReference, DifferentDurationSegmentsSharePhysicalCubic)
{
  // x(t)=t^3: first t=2s, then t=2+3s.
  const PolynomialReference ref({{2, {0, 0, 0, 8}, {0}, {0}, {0, 0.4}},
    {3, {8, 36, 54, 27}, {0}, {0}, {0.4, 0.6}}});
  EXPECT_DOUBLE_EQ(ref.duration(), 5);
  for (double t : {0.0, 1.9, 2.0, 2.1, 5.0})
  {
    FlatSample f;
    ASSERT_TRUE(ref.sample_flat(t, f));
    EXPECT_NEAR(f.p.x(), t*t*t, 1e-12);
    EXPECT_NEAR(f.v.x(), 3*t*t, 1e-12);
    EXPECT_NEAR(f.a.x(), 6*t, 1e-12);
    EXPECT_NEAR(f.j.x(), 6, 1e-12);
    EXPECT_NEAR(f.yaw_rate, 0.2, 1e-12);
  }
}

TEST(PolynomialReference, KnotUsesRightPieceAndFinalEndpointIsIncluded)
{
  Segment left = hover(1), right = hover(1);
  right.x[0] += 5e-7;  // Allowed tolerance makes right-hand selection observable.
  const PolynomialReference ref({left, right});
  FlatSample f;
  ASSERT_TRUE(ref.sample_flat(1, f));
  EXPECT_DOUBLE_EQ(f.p.x(), right.x[0]);
  ASSERT_TRUE(ref.sample_flat(2, f));
  const FlatSample saved = f;
  EXPECT_FALSE(ref.sample_flat(std::nextafter(2.0, 3.0), f));
  expect_flat_near(f, saved, 0);
  for (double t : {-1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_FALSE(ref.sample_flat(t, f));
    expect_flat_near(f, saved, 0);
  }
}

TEST(PolynomialReference, InvalidShapesNumbersAndEndpointOverflowAreRejected)
{
  EXPECT_THROW(PolynomialReference(std::vector<Segment>{}), std::invalid_argument);
  EXPECT_THROW(PolynomialReference(std::vector<Segment>(101, hover())), std::invalid_argument);
  EXPECT_NO_THROW(PolynomialReference(std::vector<Segment>(100, hover())));
  for (double d : {0.0, -1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(PolynomialReference({hover(d)}), std::invalid_argument);
    EXPECT_THROW(PolynomialReference({hover()}, false, d), std::invalid_argument);
  }
  for (int axis = 0; axis < 4; ++axis)
  {
    for (int invalid = 0; invalid < 3; ++invalid)
    {
      auto segment = hover();
      auto &coeff = axis == 0 ? segment.x : axis == 1 ? segment.y :
        axis == 2 ? segment.z : segment.yaw;
      if (invalid == 0) coeff.clear();
      if (invalid == 1) coeff.assign(11, 0);
      if (invalid == 2) coeff[0] = std::numeric_limits<double>::quiet_NaN();
      EXPECT_THROW(PolynomialReference({segment}), std::invalid_argument);
    }
  }
  auto overflow = hover(1e-200);
  // Third derivative zero is fine, but physical acceleration of s^2 overflows.
  overflow.x = {0, 0, 1};
  EXPECT_THROW(PolynomialReference({overflow}), std::invalid_argument);
  EXPECT_THROW(PolynomialReference({hover(1e308), hover(1e308)}), std::invalid_argument);
  overflow = hover();
  overflow.x = {1e308, 1e308};
  EXPECT_THROW(PolynomialReference({overflow}), std::invalid_argument);
}

TEST(PolynomialReference, EveryPositionContinuityOrderAndUnwrappedYawAreRequired)
{
  for (int axis = 0; axis < 3; ++axis)
  {
    for (int order = 0; order <= 3; ++order)
    {
      auto next = hover();
      auto &coeff = axis == 0 ? next.x : axis == 1 ? next.y : next.z;
      if (order == 0) coeff[0] += 1e-4;
      else {coeff.resize(order + 1, 0); coeff[order] = 1e-4;}
      EXPECT_THROW(PolynomialReference({hover(), next}), std::invalid_argument);
    }
  }
  auto next = hover();
  next.yaw[0] += 2 * std::acos(-1.0);
  EXPECT_THROW(PolynomialReference({hover(), next}), std::invalid_argument);
  next = hover();
  next.yaw.push_back(1e-3);
  EXPECT_THROW(PolynomialReference({hover(), next}), std::invalid_argument);
  next = hover();
  next.x[0] += 0.8e-6;
  next.y[0] += 0.8e-6;  // Norm tolerance, not per-component tolerance.
  EXPECT_THROW(PolynomialReference({hover(), next}), std::invalid_argument);
}

TEST(PolynomialReference, FinalHoldRequiresAllStationaryDerivatives)
{
  for (int order = 1; order <= 3; ++order)
  {
    auto segment = hover(1);
    // (s-1)^order isolates a nonzero endpoint derivative of this order.
    segment.x = order == 1 ? std::vector<double>{-1, 1} : order == 2 ?
      std::vector<double>{1, -2, 1} : std::vector<double>{-1, 3, -3, 1};
    EXPECT_THROW(PolynomialReference({segment}, true), std::invalid_argument);
  }
  auto segment = hover();
  segment.yaw.push_back(0.1);
  EXPECT_THROW(PolynomialReference({segment}, true), std::invalid_argument);
  // Degree-seven smoothstep has zero first through third endpoint derivatives.
  segment = hover(2);
  segment.x = {0, 0, 0, 0, 35, -84, 70, -20};
  const PolynomialReference ref({segment}, true);
  FlatSample f;
  ASSERT_TRUE(ref.sample_flat(1e100, f));
  EXPECT_DOUBLE_EQ(f.p.x(), 1);
  EXPECT_EQ(f.v.norm() + f.a.norm() + f.j.norm(), 0);
  EXPECT_EQ(f.yaw_rate, 0);
  dynamics::State state;
  dynamics::ControlInput input;
  ASSERT_TRUE(ref.sample(1e100, state, input));
  EXPECT_DOUBLE_EQ(input.thrust_acceleration, dynamics::kGravity);
  EXPECT_EQ(input.body_rates.norm(), 0);
}

TEST(PolynomialReference, FlatnessReconstructsAccelerationAndProperRotation)
{
  const PolynomialReference ref({{3, {1, 2, 0.6, 0.3}, {-2, 1, -0.8, 0.4},
    {-3, 0.1, 0.3, -0.2}, {0.2, 0.7, 0.1}}}, false, 9.7);
  for (double t : {0.0, 0.7, 2.8, 3.0})
  {
    FlatSample f;
    dynamics::State state;
    dynamics::ControlInput input;
    ASSERT_TRUE(ref.sample_flat(t, f));
    ASSERT_TRUE(ref.sample(t, state, input));
    EXPECT_LT((state.p - f.p).norm(), 1e-14);
    EXPECT_LT((state.v - f.v).norm(), 1e-14);
    EXPECT_LT((state.R.transpose()*state.R - Eigen::Matrix3d::Identity()).norm(), 1e-14);
    EXPECT_NEAR(state.R.determinant(), 1, 1e-14);
    EXPECT_LT((9.7*Vector3d::UnitZ() - input.thrust_acceleration*state.R.col(2) - f.a).norm(), 1e-13);
    EXPECT_NEAR(std::atan2(state.R(1, 0), state.R(0, 0)), f.yaw, 1e-14);
  }
}

TEST(PolynomialReference, AnalyticBodyRateMatchesRotationDerivativeOracle)
{
  const PolynomialReference ref({{3, {1, 2, 0.6, 0.3}, {-2, 1, -0.8, 0.4},
    {-3, 0.1, 0.3, -0.2}, {0.2, 0.7, 0.1}}});
  constexpr double h = 1e-5;
  for (double t : {0.1, 1.2, 2.9})
  {
    dynamics::State center, before, after;
    dynamics::ControlInput input, scratch;
    ASSERT_TRUE(ref.sample(t, center, input));
    ASSERT_TRUE(ref.sample(t-h, before, scratch));
    ASSERT_TRUE(ref.sample(t+h, after, scratch));
    const Eigen::Matrix3d body = center.R.transpose()*(after.R-before.R)/(2*h);
    const Vector3d oracle = 0.5*Vector3d(body(2,1)-body(1,2), body(0,2)-body(2,0), body(1,0)-body(0,1));
    EXPECT_LT((input.body_rates-oracle).norm(), 2e-10);
  }
  auto spin = hover(2);
  spin.yaw = {0.4, 1.2};
  dynamics::State state;
  dynamics::ControlInput input;
  ASSERT_TRUE(PolynomialReference({spin}).sample(1, state, input));
  EXPECT_LT((input.body_rates-Vector3d(0, 0, 0.6)).norm(), 1e-14);
}

TEST(PolynomialReference, SingularAndNonUprightSamplesDoNotModifyOutputs)
{
  for (double vertical_acceleration : {dynamics::kGravity, dynamics::kGravity + 1.0})
  {
    auto segment = hover(1);
    segment.z = {0, 0, vertical_acceleration/2};
    const PolynomialReference ref({segment});
    dynamics::State state;
    state.p = Vector3d(4, 5, 6);
    dynamics::ControlInput input{7, Vector3d(8, 9, 10)};
    EXPECT_FALSE(ref.sample(0.5, state, input));
    EXPECT_EQ(state.p, Vector3d(4, 5, 6));
    EXPECT_EQ(state.R, Eigen::Matrix3d::Identity());
    EXPECT_DOUBLE_EQ(input.thrust_acceleration, 7);
    EXPECT_EQ(input.body_rates, Vector3d(8, 9, 10));
  }
  auto horizontal = hover(1);
  horizontal.x = {0, 0, 1};
  horizontal.z = {0, 0, (dynamics::kGravity-1e-4)/2};
  dynamics::State state;
  dynamics::ControlInput input;
  EXPECT_FALSE(PolynomialReference({horizontal}).sample(0, state, input));
}

TEST(PolynomialReference, HorizonUsesCallerStorageAndCoversWholeWindow)
{
  const PolynomialReference ref({hover(1), hover(1)});
  std::vector<dynamics::State> states(4);
  std::vector<dynamics::ControlInput> inputs(3);
  const auto *state_storage = states.data();
  const auto *input_storage = inputs.data();
  ASSERT_TRUE(ref.sample_horizon(0.5, 0.5, states, inputs));
  EXPECT_EQ(states.data(), state_storage);
  EXPECT_EQ(inputs.data(), input_storage);
  EXPECT_EQ(states.size(), 4u);
  EXPECT_EQ(inputs.size(), 3u);
  for (std::size_t k = 0; k < states.size(); ++k)
  {
    dynamics::State expected;
    dynamics::ControlInput control;
    ASSERT_TRUE(ref.sample(0.5 + k*0.5, expected, control));
    EXPECT_EQ(states[k].p, expected.p);
    EXPECT_EQ(states[k].R, expected.R);
    if (k < inputs.size())
    {
      EXPECT_DOUBLE_EQ(inputs[k].thrust_acceleration, control.thrust_acceleration);
    }
  }
  EXPECT_FALSE(ref.sample_horizon(0.6, 0.5, states, inputs));
  for (double dt : {0.0, -0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(ref.sample_horizon(0, dt, states, inputs));
  EXPECT_FALSE(ref.sample_horizon(-0.1, 0.1, states, inputs));
  EXPECT_FALSE(ref.sample_horizon(std::numeric_limits<double>::quiet_NaN(), 0.1, states, inputs));
  states.resize(3);
  EXPECT_FALSE(ref.sample_horizon(0, 0.1, states, inputs));
  states.clear();
  EXPECT_FALSE(ref.sample_horizon(0, 0.1, states, inputs));
  inputs.clear();
  states.resize(1);
  EXPECT_FALSE(ref.sample_horizon(0, 0.1, states, inputs));
  states.resize(4);
  inputs.resize(3);
  const PolynomialReference held({hover()}, true);
  EXPECT_TRUE(held.sample_horizon(100, 0.1, states, inputs));
  EXPECT_FALSE(held.sample_horizon(1e308, 1e308, states, inputs));
  auto singular = hover();
  singular.z = {0, 0, 2*dynamics::kGravity};
  EXPECT_FALSE(PolynomialReference({singular}).sample_horizon(0, 0.1, states, inputs));
}
}  // namespace
