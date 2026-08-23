#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "policy_runtime/protocol/cia402/units.hpp"
#include "policy_runtime/robot/devices/cia402/axis.hpp"

namespace {

using policy_runtime::AxisCommand;
using policy_runtime::Cia402Axis;
using policy_runtime::Cia402PdoView;
using policy_runtime::kAxisCommandEnable;
using policy_runtime::kAxisCommandFaultReset;
using policy_runtime::kAxisCommandQuickStop;
using policy_runtime::kAxisFeedbackFollowingError;
using policy_runtime::kAxisFeedbackInvalidCommand;
using policy_runtime::kAxisFeedbackModeMismatch;
using policy_runtime::kAxisFeedbackSlewLimited;
using policy_runtime::kAxisFeedbackTargetClamped;
using policy_runtime::profiles::AxisConfig;
using policy_runtime::profiles::Cia402Mode;

AxisConfig config(Cia402Mode mode) {
  return AxisConfig{"axis", 0, 0, 0x9AU, 0x30924U, 0x10420U, mode, 1000.0,
                    -1.0, 1.0, std::chrono::milliseconds{100}, "arm", 0.25, 0.5};
}

AxisCommand enable(double target) {
  AxisCommand command{};
  command.sequence = 7;
  command.timestamp_ns = 1234;
  command.target = target;
  command.flags = kAxisCommandEnable;
  return command;
}

Cia402PdoView operational(Cia402Mode mode) {
  Cia402PdoView pdo{};
  pdo.status_word = 0x0027;
  pdo.mode_display = static_cast<std::int8_t>(mode);
  return pdo;
}

}  // namespace

TEST(Cia402UnitsTest, ConvertsEngineeringValuesWithCheckedRoundingAndRange) {
  EXPECT_EQ(policy_runtime::position_to_device_units(1.25, 1000.0), 1250);
  EXPECT_EQ(policy_runtime::position_to_device_units(-0.0015, 1000.0), -2);
  EXPECT_EQ(policy_runtime::torque_to_device_units(2.5, 100.0), 250);
  EXPECT_DOUBLE_EQ(policy_runtime::device_units_to_engineering(-125, 1000.0), -0.125);

  EXPECT_FALSE(policy_runtime::position_to_device_units(
                   std::numeric_limits<double>::quiet_NaN(), 1000.0)
                   .has_value());
  EXPECT_FALSE(policy_runtime::position_to_device_units(1.0e20, 1000.0).has_value());
  EXPECT_FALSE(policy_runtime::torque_to_device_units(40.0, 1000.0).has_value());
  EXPECT_FALSE(policy_runtime::position_to_device_units(1.0, 0.0).has_value());
}

TEST(Cia402AxisTest, ScalesFeedbackAndAppliesClampThenPerCycleSlewInCsp) {
  auto axis_config = config(Cia402Mode::csp);
  axis_config.following_error_limit = 10.0;
  Cia402Axis axis{axis_config};
  auto pdo = operational(Cia402Mode::csp);
  pdo.actual_position = 125;
  pdo.actual_velocity = -250;
  pdo.actual_torque = 50;

  auto feedback = axis.cycle(enable(2.0), pdo);

  EXPECT_EQ(pdo.control_word, 0x000F);
  EXPECT_EQ(pdo.target_position, 375);
  EXPECT_EQ(pdo.target_velocity, 0);
  EXPECT_EQ(pdo.target_torque, 0);
  EXPECT_DOUBLE_EQ(feedback.position, 0.125);
  EXPECT_DOUBLE_EQ(feedback.velocity, -0.25);
  EXPECT_DOUBLE_EQ(feedback.effort, 0.05);
  EXPECT_EQ(feedback.sequence, 7U);
  EXPECT_EQ(feedback.timestamp_ns, 1234);
  EXPECT_NE(feedback.flags & kAxisFeedbackTargetClamped, 0U);
  EXPECT_NE(feedback.flags & kAxisFeedbackSlewLimited, 0U);

  feedback = axis.cycle(enable(1.0), pdo);
  EXPECT_EQ(pdo.target_position, 625);
  EXPECT_EQ(feedback.flags & kAxisFeedbackTargetClamped, 0U);
  EXPECT_NE(feedback.flags & kAxisFeedbackSlewLimited, 0U);
}

TEST(Cia402AxisTest, WritesOnlyTheConfiguredCsvOrCstSetpoint) {
  Cia402Axis velocity_axis{config(Cia402Mode::csv)};
  auto velocity_pdo = operational(Cia402Mode::csv);
  velocity_pdo.actual_velocity = 100;
  velocity_axis.cycle(enable(0.2), velocity_pdo);
  EXPECT_EQ(velocity_pdo.target_position, 0);
  EXPECT_EQ(velocity_pdo.target_velocity, 200);
  EXPECT_EQ(velocity_pdo.target_torque, 0);

  auto torque_config = config(Cia402Mode::cst);
  torque_config.scale = 100.0;
  Cia402Axis torque_axis{torque_config};
  auto torque_pdo = operational(Cia402Mode::cst);
  torque_pdo.actual_torque = 10;
  torque_axis.cycle(enable(0.2), torque_pdo);
  EXPECT_EQ(torque_pdo.target_position, 0);
  EXPECT_EQ(torque_pdo.target_velocity, 0);
  EXPECT_EQ(torque_pdo.target_torque, 20);
}

TEST(Cia402AxisTest, NeverRequestsOperationEnabledWhenModeDisplayMismatches) {
  Cia402Axis axis{config(Cia402Mode::csp)};
  auto pdo = operational(Cia402Mode::csv);

  const auto feedback = axis.cycle(enable(0.1), pdo);

  EXPECT_EQ(pdo.control_word, 0x0002);
  EXPECT_NE(feedback.flags & kAxisFeedbackModeMismatch, 0U);
  EXPECT_FALSE(axis.verify_mode(pdo.mode_display).has_value());
}

TEST(Cia402AxisTest, QuickStopsOnFollowingErrorOrInvalidTarget) {
  auto axis_config = config(Cia402Mode::csp);
  axis_config.slew_limit = 2.0;
  axis_config.following_error_limit = 0.1;
  Cia402Axis axis{axis_config};
  auto pdo = operational(Cia402Mode::csp);

  auto feedback = axis.cycle(enable(0.2), pdo);
  EXPECT_EQ(pdo.control_word, 0x0002);
  EXPECT_NE(feedback.flags & kAxisFeedbackFollowingError, 0U);

  feedback = axis.cycle(enable(std::numeric_limits<double>::quiet_NaN()), pdo);
  EXPECT_EQ(pdo.control_word, 0x0002);
  EXPECT_NE(feedback.flags & kAxisFeedbackInvalidCommand, 0U);
}

TEST(Cia402AxisTest, TreatsNegativeInfiniteSafetyLimitsAsInvalid) {
  auto axis_config = config(Cia402Mode::csp);
  axis_config.slew_limit = -std::numeric_limits<double>::infinity();
  Cia402Axis axis{axis_config};
  auto pdo = operational(Cia402Mode::csp);

  const auto feedback = axis.cycle(enable(0.2), pdo);

  EXPECT_EQ(pdo.control_word, 0x0002);
  EXPECT_NE(feedback.flags & kAxisFeedbackInvalidCommand, 0U);
}

TEST(Cia402AxisTest, RejectsAmbiguousFlagsAndAllowsExplicitFaultReset) {
  Cia402Axis axis{config(Cia402Mode::csp)};
  auto pdo = operational(Cia402Mode::csp);
  auto command = enable(0.0);
  command.flags |= kAxisCommandQuickStop;

  auto feedback = axis.cycle(command, pdo);
  EXPECT_EQ(pdo.control_word, 0x0002);
  EXPECT_NE(feedback.flags & kAxisFeedbackInvalidCommand, 0U);

  pdo.status_word = 0x0008;
  command.flags = kAxisCommandFaultReset;
  feedback = axis.cycle(command, pdo);
  EXPECT_EQ(pdo.control_word, 0x0080);
  EXPECT_EQ(feedback.flags & kAxisFeedbackInvalidCommand, 0U);
}
