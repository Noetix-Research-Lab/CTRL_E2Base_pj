#include "rl_controllers/ControlStateMachine.h"

#include <gtest/gtest.h>

namespace legged
{
TEST(ControlStateMachineTest, StartAndStopAreDeterministic)
{
  const auto start = applyControlRequest(ControlMode::DEFAULT, false, ControlRequest::START);
  EXPECT_TRUE(start.accepted);
  EXPECT_TRUE(start.startControl);
  EXPECT_EQ(start.mode, ControlMode::LIE);
  EXPECT_EQ(start.event, ControlEvent::START_CONTROL);
  EXPECT_TRUE(start.resetStandTransition);
  EXPECT_TRUE(start.captureCurrentJointAngles);

  const auto stop = applyControlRequest(ControlMode::WALK, true, ControlRequest::START);
  EXPECT_TRUE(stop.accepted);
  EXPECT_FALSE(stop.startControl);
  EXPECT_EQ(stop.mode, ControlMode::DEFAULT);
  EXPECT_EQ(stop.event, ControlEvent::SHUTDOWN_CONTROL);
}

TEST(ControlStateMachineTest, LocomotionTransitionsPreserveExistingRules)
{
  EXPECT_EQ(applyControlRequest(ControlMode::STAND, true, ControlRequest::WALK).mode,
            ControlMode::WALK);
  EXPECT_EQ(applyControlRequest(ControlMode::WALK, true, ControlRequest::DANCE).mode,
            ControlMode::DANCE);
  EXPECT_EQ(applyControlRequest(ControlMode::DANCE, true, ControlRequest::WALK).mode,
            ControlMode::WALK);
  EXPECT_EQ(applyControlRequest(ControlMode::WALK, true, ControlRequest::DOWN).mode,
            ControlMode::DOWN);
  EXPECT_EQ(applyControlRequest(ControlMode::DOWN, true, ControlRequest::WALK).event,
            ControlEvent::DOWN_TO_WALK);
  const auto upFromWalk =
      applyControlRequest(ControlMode::WALK, true, ControlRequest::UP);
  EXPECT_FALSE(upFromWalk.accepted);
  EXPECT_EQ(upFromWalk.mode, ControlMode::WALK);
  const auto upFromDown =
      applyControlRequest(ControlMode::DOWN, true, ControlRequest::UP);
  EXPECT_TRUE(upFromDown.accepted);
  EXPECT_EQ(upFromDown.mode, ControlMode::UP);
  EXPECT_EQ(upFromDown.event, ControlEvent::DOWN_TO_UP);
  EXPECT_EQ(applyControlRequest(ControlMode::UP, true, ControlRequest::WALK).event,
            ControlEvent::UP_TO_WALK);
}

TEST(ControlStateMachineTest, InvalidTransitionsHaveNoSideEffects)
{
  const auto transition =
      applyControlRequest(ControlMode::LIE, true, ControlRequest::DANCE);
  EXPECT_FALSE(transition.accepted);
  EXPECT_EQ(transition.mode, ControlMode::LIE);
  EXPECT_TRUE(transition.startControl);
  EXPECT_EQ(transition.event, ControlEvent::COUNT);
  EXPECT_FALSE(transition.resetObservation);
}

TEST(ControlStateMachineTest, PositionRequestMovesOnlyTowardSafeModes)
{
  const auto fromWalk =
      applyControlRequest(ControlMode::WALK, true, ControlRequest::POSITION);
  EXPECT_EQ(fromWalk.mode, ControlMode::STAND);
  EXPECT_EQ(fromWalk.event, ControlEvent::TO_STAND);

  const auto fromDefault =
      applyControlRequest(ControlMode::DEFAULT, false, ControlRequest::POSITION);
  EXPECT_EQ(fromDefault.mode, ControlMode::LIE);
  EXPECT_EQ(fromDefault.event, ControlEvent::DEFAULT_TO_LIE);
  EXPECT_TRUE(fromDefault.captureCurrentJointAngles);
}

TEST(ControlStateMachineTest, JoystickDisconnectFallsBackByMode)
{
  const auto dance = joystickDisconnectFallback(ControlMode::DANCE, true);
  EXPECT_EQ(dance.mode, ControlMode::WALK);
  EXPECT_TRUE(dance.startControl);
  EXPECT_TRUE(dance.resetObservation);
  EXPECT_EQ(dance.event, ControlEvent::JOYSTICK_DISCONNECTED);

  const auto walk = joystickDisconnectFallback(ControlMode::WALK, true);
  EXPECT_EQ(walk.mode, ControlMode::WALK);
  EXPECT_TRUE(walk.startControl);

  for (const auto mode : {ControlMode::DOWN, ControlMode::UP}) {
    const auto fallback = joystickDisconnectFallback(mode, true);
    EXPECT_EQ(fallback.mode, ControlMode::WALK);
    EXPECT_TRUE(fallback.startControl);
    EXPECT_TRUE(fallback.resetObservation);
  }

  for (const auto mode : {ControlMode::LIE, ControlMode::STAND,
                          ControlMode::DEFAULT}) {
    const auto fallback = joystickDisconnectFallback(mode, true);
    EXPECT_EQ(fallback.mode, ControlMode::DEFAULT);
    EXPECT_FALSE(fallback.startControl);
  }
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
