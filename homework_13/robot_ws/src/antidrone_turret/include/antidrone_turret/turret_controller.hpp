#pragma once

#include <cstdint>
#include <string>

namespace antidrone_turret {

// Gimbal direction constants (vertical aim)
enum class GimbalDirection : std::int8_t {
  kDown = -1,
  kCenter = 0,
  kUp = 1,
};

// Servo direction constants (horizontal aim)
enum class ServoDirection : std::int8_t {
  kLeft = -1,
  kCenter = 0,
  kRight = 1,
};

// Target assessment states
enum class TargetState : std::uint8_t {
  kNone = 0,
  kLowConfidence = 1,
  kLocked = 2,
};

// Controller action states
enum class ActionState : std::uint8_t {
  kIdle = 0,
  kTrack = 1,
};

// Trigger decision states
enum class TriggerState : std::uint8_t {
  kSkip = 0,
  kRequested = 1,
  kReloading = 2,
};

// GimbalCommand message-like struct (pure C++)
struct GimbalCommand {
  std::int8_t direction{0};
  float target_y{0.0f};
  float error_y{0.0f};
};

// ServoCommand message-like struct (pure C++)
struct ServoCommand {
  std::int8_t direction{0};
  float target_x{0.0f};
  float error_x{0.0f};
};

// TurretStatus message-like struct (pure C++)
struct TurretStatus {
  std::uint8_t target_state{0};
  std::uint8_t action{0};
  std::uint8_t trigger_state{0};
  float confidence{0.0f};
  float distance_m{0.0f};
};

// Target input from perception
struct TargetInput {
  bool visible{false};
  float x{0.0f};
  float y{0.0f};
  float distance_m{0.0f};
  float confidence{0.0f};
};

// Actuator state input
struct ActuatorInput {
  bool ready{true};
  std::uint32_t trigger_count{0};
};

// Controller configuration
struct ControllerConfig {
  float confidence_threshold{0.80f};
  float max_distance_m{30.0f};
};

/**
 * Assess target quality based on visibility and confidence.
 * Returns TARGET_NONE if not visible, TARGET_LOW_CONFIDENCE if below threshold,
 * TARGET_LOCKED otherwise.
 */
inline TargetState assess_target(bool visible, float confidence, float confidence_threshold)
{
  if (!visible) {
    return TargetState::kNone;
  }
  if (confidence < confidence_threshold) {
    return TargetState::kLowConfidence;
  }
  return TargetState::kLocked;
}

/**
 * Determine controller action based on target state.
 * ACTION_IDLE for non-visible or low-confidence targets, ACTION_TRACK for locked targets.
 */
inline ActionState determine_action(TargetState target_state)
{
  if (target_state == TargetState::kLocked) {
    return ActionState::kTrack;
  }
  return ActionState::kIdle;
}

/**
 * Build GimbalCommand from target y-coordinate.
 * error_y = 240.0 - y (camera y grows downward, so UP means error > 0).
 */
inline GimbalCommand build_gimbal_command(float target_y)
{
  GimbalCommand cmd;
  cmd.target_y = target_y;
  cmd.error_y = 240.0f - target_y;

  if (cmd.error_y > 0.0f) {
    cmd.direction = static_cast<std::int8_t>(GimbalDirection::kUp);
  } else if (cmd.error_y < 0.0f) {
    cmd.direction = static_cast<std::int8_t>(GimbalDirection::kDown);
  } else {
    cmd.direction = static_cast<std::int8_t>(GimbalDirection::kCenter);
  }

  return cmd;
}

/**
 * Build ServoCommand from target x-coordinate.
 * error_x = x - 320.0 (camera x grows rightward).
 */
inline ServoCommand build_servo_command(float target_x)
{
  ServoCommand cmd;
  cmd.target_x = target_x;
  cmd.error_x = target_x - 320.0f;

  if (cmd.error_x > 0.0f) {
    cmd.direction = static_cast<std::int8_t>(ServoDirection::kRight);
  } else if (cmd.error_x < 0.0f) {
    cmd.direction = static_cast<std::int8_t>(ServoDirection::kLeft);
  } else {
    cmd.direction = static_cast<std::int8_t>(ServoDirection::kCenter);
  }

  return cmd;
}

/**
 * Decide trigger action based on distance and actuator state.
 * TRIGGER_REQUESTED if close enough and actuator is READY.
 * TRIGGER_RELOADING if close enough but actuator is RELOADING.
 * TRIGGER_SKIP otherwise.
 */
inline TriggerState decide_trigger(float distance_m, float max_distance_m, bool actuator_ready)
{
  if (distance_m > max_distance_m) {
    return TriggerState::kSkip;
  }
  if (actuator_ready) {
    return TriggerState::kRequested;
  }
  // Distance is within range but actuator is not ready (RELOADING).
  return TriggerState::kReloading;
}

/**
 * Build complete TurretStatus from all components.
 */
inline TurretStatus build_turret_status(
    TargetState target_state,
    ActionState action,
    TriggerState trigger_state,
    float confidence,
    float distance_m)
{
  TurretStatus status;
  status.target_state = static_cast<std::uint8_t>(target_state);
  status.action = static_cast<std::uint8_t>(action);
  status.trigger_state = static_cast<std::uint8_t>(trigger_state);
  status.confidence = confidence;
  status.distance_m = distance_m;
  return status;
}

/**
 * Main controller process: takes target input and actuator state,
 * returns all outputs (gimbal command, servo command, turret status).
 * Returns std::nullopt for gimbal/servo commands when action is IDLE.
 */
struct ControllerOutput {
  GimbalCommand gimbal_cmd;
  ServoCommand servo_cmd;
  TurretStatus status;
  bool should_trigger{false};
  float trigger_confidence{0.0f};
  float trigger_distance{0.0f};
};

inline ControllerOutput process_controller(
    const TargetInput& target,
    const ActuatorInput& actuator,
    const ControllerConfig& config)
{
  ControllerOutput output{};

  // Assess target
  const auto target_state = assess_target(target.visible, target.confidence, config.confidence_threshold);

  // Determine action
  const auto action = determine_action(target_state);

  // Decide trigger
  TriggerState trigger_state;
  if (target_state == TargetState::kLocked) {
    trigger_state = decide_trigger(target.distance_m, config.max_distance_m, actuator.ready);
  } else {
    trigger_state = TriggerState::kSkip;
  }

  // Build commands
  output.gimbal_cmd = build_gimbal_command(target.y);
  output.servo_cmd = build_servo_command(target.x);
  output.status = build_turret_status(target_state, action, trigger_state, target.confidence, target.distance_m);

  // Set trigger flag
  if (trigger_state == TriggerState::kRequested) {
    output.should_trigger = true;
    output.trigger_confidence = target.confidence;
    output.trigger_distance = target.distance_m;
  }

  return output;
}

}  // namespace antidrone_turret
