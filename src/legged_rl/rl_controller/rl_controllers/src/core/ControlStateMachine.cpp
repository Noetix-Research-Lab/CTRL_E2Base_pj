#include "rl_controllers/ControlStateMachine.h"

namespace legged
{
namespace
{
ControlTransition transition(ControlMode from, bool started, ControlMode to,
                             ControlEvent event) noexcept
{
  ControlTransition result{from, event, started};
  result.mode = to;
  result.accepted = true;
  return result;
}
}  // namespace

ControlTransition applyControlRequest(ControlMode mode, bool startControl,
                                      ControlRequest request) noexcept
{
  ControlTransition result{mode, ControlEvent::COUNT, startControl};
  switch (request) {
    case ControlRequest::START:
      if (!startControl) {
        result = transition(mode, false, ControlMode::LIE, ControlEvent::START_CONTROL);
        result.startControl = true;
        result.resetStandTransition = true;
        result.captureCurrentJointAngles = true;
      } else {
        result = transition(mode, true, ControlMode::DEFAULT, ControlEvent::SHUTDOWN_CONTROL);
        result.startControl = false;
      }
      break;
    case ControlRequest::SWITCH_MODE:
      if (startControl && mode == ControlMode::STAND) {
        result = transition(mode, startControl, ControlMode::LIE, ControlEvent::STAND_TO_LIE);
      } else if (startControl && mode == ControlMode::LIE) {
        result = transition(mode, startControl, ControlMode::STAND, ControlEvent::LIE_TO_STAND);
      }
      if (result.accepted) {
        result.resetStandTransition = true;
        result.captureCurrentJointAngles = true;
      }
      break;
    case ControlRequest::LIE_TO_STAND:
      if (startControl && mode == ControlMode::LIE) {
        result = transition(mode, startControl, ControlMode::STAND, ControlEvent::LIE_TO_STAND);
        result.resetStandTransition = true;
        result.captureCurrentJointAngles = true;
      }
      break;
    case ControlRequest::STAND_TO_LIE:
      if (startControl && mode == ControlMode::STAND) {
        result = transition(mode, startControl, ControlMode::LIE, ControlEvent::STAND_TO_LIE);
        result.resetStandTransition = true;
        result.captureCurrentJointAngles = true;
      }
      break;
    case ControlRequest::WALK:
      if (mode == ControlMode::STAND) {
        result = transition(mode, startControl, ControlMode::WALK, ControlEvent::STAND_TO_WALK);
      } else if (mode == ControlMode::DANCE) {
        result = transition(mode, startControl, ControlMode::WALK, ControlEvent::DANCE_TO_WALK);
      } else if (mode == ControlMode::DOWN) {
        result = transition(mode, startControl, ControlMode::WALK, ControlEvent::DOWN_TO_WALK);
      } else if (mode == ControlMode::UP) {
        result = transition(mode, startControl, ControlMode::WALK, ControlEvent::UP_TO_WALK);
      }
      result.resetObservation = result.accepted;
      break;
    case ControlRequest::WALK_TO_STAND:
    case ControlRequest::POSITION:
      if (mode == ControlMode::WALK || mode == ControlMode::DANCE ||
          mode == ControlMode::DOWN || mode == ControlMode::UP) {
        result = transition(mode, startControl, ControlMode::STAND, ControlEvent::TO_STAND);
      } else if (request == ControlRequest::POSITION && mode == ControlMode::DEFAULT) {
        result = transition(mode, startControl, ControlMode::LIE, ControlEvent::DEFAULT_TO_LIE);
        result.resetStandTransition = true;
        result.captureCurrentJointAngles = true;
      }
      break;
    case ControlRequest::DANCE:
      if (mode == ControlMode::WALK) {
        result = transition(mode, startControl, ControlMode::DANCE, ControlEvent::WALK_TO_DANCE);
        result.resetObservation = true;
        result.resetDanceTime = true;
      }
      break;
    case ControlRequest::DOWN:
      if (mode == ControlMode::WALK) {
        result = transition(mode, startControl, ControlMode::DOWN, ControlEvent::WALK_TO_DOWN);
        result.resetObservation = true;
        result.resetDanceTime = true;
      }
      break;
    case ControlRequest::UP:
      if (mode == ControlMode::DOWN) {
        result = transition(mode, startControl, ControlMode::UP, ControlEvent::DOWN_TO_UP);
        result.resetObservation = true;
        result.resetDanceTime = true;
      }
      break;
    case ControlRequest::COUNT:
      break;
  }
  return result;
}

ControlTransition joystickDisconnectFallback(ControlMode mode,
                                              bool startControl) noexcept
{
  ControlTransition result{mode, ControlEvent::JOYSTICK_DISCONNECTED, startControl};
  result.accepted = true;
  if (mode == ControlMode::DANCE || mode == ControlMode::DOWN ||
      mode == ControlMode::UP) {
    result.mode = ControlMode::WALK;
    result.resetObservation = true;
  } else if (mode != ControlMode::WALK) {
    result.mode = ControlMode::DEFAULT;
    result.startControl = false;
  }
  return result;
}
}  // namespace legged
