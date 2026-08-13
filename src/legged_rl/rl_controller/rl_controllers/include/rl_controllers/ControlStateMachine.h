#pragma once

#include <cstddef>
#include <cstdint>

namespace legged
{
enum class ControlMode : uint8_t
{
  LIE = 0,
  STAND = 1,
  WALK = 2,
  DANCE = 3,
  DEFAULT = 4,
  DOWN = 5,
  UP = 6
};

enum class ControlRequest : size_t
{
  START,
  SWITCH_MODE,
  LIE_TO_STAND,
  STAND_TO_LIE,
  WALK,
  WALK_TO_STAND,
  DANCE,
  DOWN,
  UP,
  POSITION,
  COUNT
};

enum class ControlEvent : uint8_t
{
  EMERGENCY_STOP,
  CMD_VEL_TIMEOUT,
  START_CONTROL,
  SHUTDOWN_CONTROL,
  STAND_TO_LIE,
  LIE_TO_STAND,
  STAND_TO_WALK,
  DANCE_TO_WALK,
  DOWN_TO_WALK,
  UP_TO_WALK,
  TO_STAND,
  WALK_TO_DANCE,
  WALK_TO_DOWN,
  DOWN_TO_UP,
  DEFAULT_TO_LIE,
  UNEXPECTED_MODE,
  FALL_PROTECTION,
  WALK_ACTION_TIMEOUT,
  DANCE_ACTION_TIMEOUT,
  DOWN_ACTION_TIMEOUT,
  UP_ACTION_TIMEOUT,
  WALK_POLICY_ERROR,
  DANCE_POLICY_ERROR,
  DOWN_POLICY_ERROR,
  UP_POLICY_ERROR,
  OBSERVATION_BUFFER_OVERFLOW,
  ACTION_SIZE_MISMATCH,
  WALK_PROPRIO_MISMATCH,
  DANCE_PROPRIO_MISMATCH,
  DOWN_PROPRIO_MISMATCH,
  UP_PROPRIO_MISMATCH,
  DANCE_COMPLETE,
  DOWN_COMPLETE,
  UP_COMPLETE,
  EMERGENCY_STOP_RESET,
  CONTROL_REQUEST_CONFLICT,
  JOYSTICK_DISCONNECTED,
  COUNT
};

struct ControlTransition
{
  ControlMode mode;
  ControlEvent event{ControlEvent::COUNT};
  bool startControl;
  bool accepted{false};
  bool resetStandTransition{false};
  bool captureCurrentJointAngles{false};
  bool resetObservation{false};
  bool resetDanceTime{false};
};

// Pure, allocation-free transition reducer. ROS callbacks and debounce policy
// remain at the adapter boundary; all mode validity rules live here.
ControlTransition applyControlRequest(ControlMode mode, bool startControl,
                                      ControlRequest request) noexcept;
ControlTransition joystickDisconnectFallback(ControlMode mode,
                                              bool startControl) noexcept;
}  // namespace legged
