#include <gtest/gtest.h>

#include "antidrone_turret/turret_controller.hpp"

namespace {

// ============================================================
// Target assessment tests
// ============================================================

TEST(TurretControllerTest, AssessTargetNoneWhenNotVisible)
{
  const auto state = antidrone_turret::assess_target(false, 0.95f, 0.80f);
  EXPECT_EQ(state, antidrone_turret::TargetState::kNone);
}

TEST(TurretControllerTest, AssessTargetLowConfidenceWhenBelowThreshold)
{
  const auto state = antidrone_turret::assess_target(true, 0.79f, 0.80f);
  EXPECT_EQ(state, antidrone_turret::TargetState::kLowConfidence);
}

TEST(TurretControllerTest, AssessTargetLockedWhenAtThreshold)
{
  const auto state = antidrone_turret::assess_target(true, 0.80f, 0.80f);
  EXPECT_EQ(state, antidrone_turret::TargetState::kLocked);
}

TEST(TurretControllerTest, AssessTargetLockedWhenAboveThreshold)
{
  const auto state = antidrone_turret::assess_target(true, 0.95f, 0.80f);
  EXPECT_EQ(state, antidrone_turret::TargetState::kLocked);
}

// ============================================================
// Action determination tests
// ============================================================

TEST(TurretControllerTest, DetermineActionIdleForNone)
{
  const auto action = antidrone_turret::determine_action(
    antidrone_turret::TargetState::kNone);
  EXPECT_EQ(action, antidrone_turret::ActionState::kIdle);
}

TEST(TurretControllerTest, DetermineActionIdleForLowConfidence)
{
  const auto action = antidrone_turret::determine_action(
    antidrone_turret::TargetState::kLowConfidence);
  EXPECT_EQ(action, antidrone_turret::ActionState::kIdle);
}

TEST(TurretControllerTest, DetermineActionTrackForLocked)
{
  const auto action = antidrone_turret::determine_action(
    antidrone_turret::TargetState::kLocked);
  EXPECT_EQ(action, antidrone_turret::ActionState::kTrack);
}

// ============================================================
// Servo command tests
// ============================================================

TEST(TurretControllerTest, ServoCommandRightWhenXGreaterThan320)
{
  const auto cmd = antidrone_turret::build_servo_command(420.0f);
  EXPECT_EQ(cmd.direction, static_cast<std::int8_t>(antidrone_turret::ServoDirection::kRight));
  EXPECT_FLOAT_EQ(cmd.target_x, 420.0f);
  EXPECT_FLOAT_EQ(cmd.error_x, 100.0f);
}

TEST(TurretControllerTest, ServoCommandLeftWhenXLessThan320)
{
  const auto cmd = antidrone_turret::build_servo_command(220.0f);
  EXPECT_EQ(cmd.direction, static_cast<std::int8_t>(antidrone_turret::ServoDirection::kLeft));
  EXPECT_FLOAT_EQ(cmd.target_x, 220.0f);
  EXPECT_FLOAT_EQ(cmd.error_x, -100.0f);
}

TEST(TurretControllerTest, ServoCommandCenterWhenXEquals320)
{
  const auto cmd = antidrone_turret::build_servo_command(320.0f);
  EXPECT_EQ(cmd.direction, static_cast<std::int8_t>(antidrone_turret::ServoDirection::kCenter));
  EXPECT_FLOAT_EQ(cmd.target_x, 320.0f);
  EXPECT_FLOAT_EQ(cmd.error_x, 0.0f);
}

// ============================================================
// Gimbal command tests
// ============================================================

TEST(TurretControllerTest, GimbalCommandUpWhenYLessThan240)
{
  const auto cmd = antidrone_turret::build_gimbal_command(180.0f);
  EXPECT_EQ(cmd.direction, static_cast<std::int8_t>(antidrone_turret::GimbalDirection::kUp));
  EXPECT_FLOAT_EQ(cmd.target_y, 180.0f);
  EXPECT_FLOAT_EQ(cmd.error_y, 60.0f);
}

TEST(TurretControllerTest, GimbalCommandDownWhenYGreaterThan240)
{
  const auto cmd = antidrone_turret::build_gimbal_command(300.0f);
  EXPECT_EQ(cmd.direction, static_cast<std::int8_t>(antidrone_turret::GimbalDirection::kDown));
  EXPECT_FLOAT_EQ(cmd.target_y, 300.0f);
  EXPECT_FLOAT_EQ(cmd.error_y, -60.0f);
}

TEST(TurretControllerTest, GimbalCommandCenterWhenYEquals240)
{
  const auto cmd = antidrone_turret::build_gimbal_command(240.0f);
  EXPECT_EQ(cmd.direction, static_cast<std::int8_t>(antidrone_turret::GimbalDirection::kCenter));
  EXPECT_FLOAT_EQ(cmd.target_y, 240.0f);
  EXPECT_FLOAT_EQ(cmd.error_y, 0.0f);
}

// ============================================================
// Trigger decision tests
// ============================================================

TEST(TurretControllerTest, TriggerRequestedWhenCloseAndReady)
{
  const auto state = antidrone_turret::decide_trigger(25.0f, 30.0f, true);
  EXPECT_EQ(state, antidrone_turret::TriggerState::kRequested);
}

TEST(TurretControllerTest, TriggerRequestedAtExactMaxDistance)
{
  const auto state = antidrone_turret::decide_trigger(30.0f, 30.0f, true);
  EXPECT_EQ(state, antidrone_turret::TriggerState::kRequested);
}

TEST(TurretControllerTest, TriggerSkipWhenFar)
{
  const auto state = antidrone_turret::decide_trigger(31.0f, 30.0f, true);
  EXPECT_EQ(state, antidrone_turret::TriggerState::kSkip);
}

TEST(TurretControllerTest, TriggerReloadingWhenCloseAndNotReady)
{
  const auto state = antidrone_turret::decide_trigger(25.0f, 30.0f, false);
  EXPECT_EQ(state, antidrone_turret::TriggerState::kReloading);
}

TEST(TurretControllerTest, TriggerSkipWhenFarAndNotReady)
{
  const auto state = antidrone_turret::decide_trigger(50.0f, 30.0f, false);
  EXPECT_EQ(state, antidrone_turret::TriggerState::kSkip);
}

// ============================================================
// Turret status tests
// ============================================================

TEST(TurretControllerTest, StatusForFarValidTarget)
{
  const antidrone_turret::TargetInput target{true, 400.0f, 200.0f, 50.0f, 0.90f};
  const antidrone_turret::ActuatorInput actuator{true, 0};
  const antidrone_turret::ControllerConfig config{0.80f, 30.0f};

  const auto output = antidrone_turret::process_controller(target, actuator, config);

  EXPECT_EQ(output.status.target_state, static_cast<std::uint8_t>(antidrone_turret::TargetState::kLocked));
  EXPECT_EQ(output.status.action, static_cast<std::uint8_t>(antidrone_turret::ActionState::kTrack));
  EXPECT_EQ(output.status.trigger_state, static_cast<std::uint8_t>(antidrone_turret::TriggerState::kSkip));
  EXPECT_FALSE(output.should_trigger);
}

TEST(TurretControllerTest, StatusForCloseValidTargetWithReadyActuator)
{
  const antidrone_turret::TargetInput target{true, 400.0f, 200.0f, 25.0f, 0.90f};
  const antidrone_turret::ActuatorInput actuator{true, 0};
  const antidrone_turret::ControllerConfig config{0.80f, 30.0f};

  const auto output = antidrone_turret::process_controller(target, actuator, config);

  EXPECT_EQ(output.status.target_state, static_cast<std::uint8_t>(antidrone_turret::TargetState::kLocked));
  EXPECT_EQ(output.status.action, static_cast<std::uint8_t>(antidrone_turret::ActionState::kTrack));
  EXPECT_EQ(output.status.trigger_state, static_cast<std::uint8_t>(antidrone_turret::TriggerState::kRequested));
  EXPECT_TRUE(output.should_trigger);
  EXPECT_FLOAT_EQ(output.trigger_confidence, 0.90f);
  EXPECT_FLOAT_EQ(output.trigger_distance, 25.0f);
}

TEST(TurretControllerTest, StatusForCloseValidTargetWithReloadingActuator)
{
  const antidrone_turret::TargetInput target{true, 400.0f, 200.0f, 25.0f, 0.90f};
  const antidrone_turret::ActuatorInput actuator{false, 1};
  const antidrone_turret::ControllerConfig config{0.80f, 30.0f};

  const auto output = antidrone_turret::process_controller(target, actuator, config);

  EXPECT_EQ(output.status.target_state, static_cast<std::uint8_t>(antidrone_turret::TargetState::kLocked));
  EXPECT_EQ(output.status.action, static_cast<std::uint8_t>(antidrone_turret::ActionState::kTrack));
  EXPECT_EQ(output.status.trigger_state, static_cast<std::uint8_t>(antidrone_turret::TriggerState::kReloading));
  EXPECT_FALSE(output.should_trigger);
}

TEST(TurretControllerTest, StatusForLowConfidenceTarget)
{
  const antidrone_turret::TargetInput target{true, 400.0f, 200.0f, 25.0f, 0.75f};
  const antidrone_turret::ActuatorInput actuator{true, 0};
  const antidrone_turret::ControllerConfig config{0.80f, 30.0f};

  const auto output = antidrone_turret::process_controller(target, actuator, config);

  EXPECT_EQ(output.status.target_state, static_cast<std::uint8_t>(antidrone_turret::TargetState::kLowConfidence));
  EXPECT_EQ(output.status.action, static_cast<std::uint8_t>(antidrone_turret::ActionState::kIdle));
  EXPECT_EQ(output.status.trigger_state, static_cast<std::uint8_t>(antidrone_turret::TriggerState::kSkip));
  EXPECT_FALSE(output.should_trigger);
}

TEST(TurretControllerTest, StatusForInvisibleTarget)
{
  const antidrone_turret::TargetInput target{false, 0.0f, 0.0f, 0.0f, 0.0f};
  const antidrone_turret::ActuatorInput actuator{true, 0};
  const antidrone_turret::ControllerConfig config{0.80f, 30.0f};

  const auto output = antidrone_turret::process_controller(target, actuator, config);

  EXPECT_EQ(output.status.target_state, static_cast<std::uint8_t>(antidrone_turret::TargetState::kNone));
  EXPECT_EQ(output.status.action, static_cast<std::uint8_t>(antidrone_turret::ActionState::kIdle));
  EXPECT_EQ(output.status.trigger_state, static_cast<std::uint8_t>(antidrone_turret::TriggerState::kSkip));
  EXPECT_FALSE(output.should_trigger);
}

}  // namespace
