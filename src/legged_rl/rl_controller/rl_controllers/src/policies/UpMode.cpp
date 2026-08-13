#include "rl_controllers/AcController.h"

#include <algorithm>

namespace legged
{
void AcController::handleUpMode()
{
  enterPolicyMode(PolicyKind::UP);
  if (handleUpPreparation()) return;

  handleMotionMode(PolicyKind::UP, up_,
                   ControlEvent::UP_COMPLETE, ControlEvent::UP_POLICY_ERROR,
                   ControlEvent::UP_ACTION_TIMEOUT, ControlEvent::UP_PROPRIO_MISMATCH);
}

void AcController::beginUpPreparation() noexcept
{
  resetUpPlaybackClock();
  for (int joint = 0; joint < actionsSizeDance_; ++joint) {
    upTransitionStartAngles_(joint) = policyJointHandles_[joint].getPosition();
  }
  upTransitionStartNs_ = 0;
  upGainsApplied_ = false;
  upPreparationActive_ = true;
}

bool AcController::handleUpPreparation() noexcept
{
  if (!upPreparationActive_) return false;

  const auto& runtime = up_.policy.runtime();
  const auto& actionScale = up_.policy.metadata().actionScale();
  const auto& stiffness = up_.policy.metadata().stiffness();
  const auto& damping = up_.policy.metadata().damping();
  if (runtime.jointPositionFrames.empty() ||
      runtime.jointPositionFrames.front().size() != static_cast<size_t>(actionsSizeDance_) ||
      actionScale.size() != static_cast<size_t>(actionsSizeDance_) ||
      stiffness.size() != static_cast<size_t>(actionsSizeDance_) ||
      damping.size() != static_cast<size_t>(actionsSizeDance_) ||
      upTransitionStartAngles_.size() != actionsSizeDance_) {
    enqueueControlEvent(ControlEvent::UP_PROPRIO_MISMATCH);
    upPreparationActive_ = false;
    leavePolicyMode();
    mode_ = Mode::DEFAULT;
    RLControllerBase::handleDefaultMode();
    return true;
  }

  const int64_t nowNs = steadyNowNs();
  if (!upGainsApplied_) {
    // First take ownership at the measured pose. This applies the UP gains
    // without introducing a simultaneous desired-position step.
    for (int joint = 0; joint < actionsSizeDance_; ++joint) {
      policyJointHandles_[joint].setCommand(
          upTransitionStartAngles_(joint), 0, stiffness[joint], damping[joint], 0);
    }
    upTransitionStartNs_ = nowNs;
    upGainsApplied_ = true;
    ROS_INFO("UP preparation: gains applied; interpolating to motion frame 0");
    return true;
  }

  const int64_t elapsedNs = nowNs >= upTransitionStartNs_
      ? nowNs - upTransitionStartNs_ : 0;
  const double blend = upTransitionDurationNs_ <= 0
      ? 1.0
      : std::clamp(static_cast<double>(elapsedNs) /
                       static_cast<double>(upTransitionDurationNs_),
                   0.0, 1.0);
  const auto& firstFrame = runtime.jointPositionFrames.front();
  for (int joint = 0; joint < actionsSizeDance_; ++joint) {
    const double desired = upTransitionStartAngles_(joint) * (1.0 - blend) +
                           firstFrame[joint] * blend;
    policyJointHandles_[joint].setCommand(
        desired, 0, stiffness[joint], damping[joint], 0);
  }

  if (blend >= 1.0) {
    // Until the first inference result arrives, make the normal policy command
    // path continue holding frame zero instead of falling back to zero action
    // (which would command the policy's default pose for one or more cycles).
    for (int joint = 0; joint < actionsSizeDance_; ++joint) {
      if (actionScale[joint] == 0.0) {
        enqueueControlEvent(ControlEvent::UP_POLICY_ERROR);
        upPreparationActive_ = false;
        leavePolicyMode();
        mode_ = Mode::DEFAULT;
        RLControllerBase::handleDefaultMode();
        return true;
      }
      up_.actions[joint] = static_cast<tensor_element_t>(
          (firstFrame[joint] - up_.defaultJointAngles(joint)) / actionScale[joint]);
      upPolicyBlendStartActions_(joint) = up_.actions[joint];
      up_.lastActions(joint) = up_.actions[joint];
    }
    upPreparationActive_ = false;
    resetMotionPlaybackClock(up_);
    upPolicyBlendStartNs_ = nowNs;
    upPolicyBlendActive_ = true;
    // The action timeout starts when policy execution starts, not while the
    // deterministic pose transition is in progress.
    policyEntryTimestampNs_ = nowNs;
    lastAppliedObservationTimestampNs_ = 0;
    ROS_INFO("UP preparation complete: blending frame-0 hold into policy output");
  }
  return true;
}

bool AcController::runUpPolicy(const ObservationPacket& input, ActionPacket& output)
{
  return runMotionPolicy(up_, ControlEvent::UP_POLICY_ERROR, input, output);
}

void AcController::resetUpPlaybackClock() noexcept
{
  resetMotionPlaybackClock(up_);
}

template<>
struct PolicyModeRegistrationBuilder<PolicyKind::UP>
{
  static bool run(AcController& controller, const ObservationPacket& input,
                  ActionPacket& output)
  {
    return controller.runUpPolicy(input, output);
  }
  static std::vector<float>& actions(AcController& controller) { return controller.up_.actions; }
  static vector_t& lastActions(AcController& controller) { return controller.up_.lastActions; }
  static void enter(AcController& controller) { controller.beginUpPreparation(); }
  static bool loadMotion(AcController& controller, ros::NodeHandle& nh)
  {
    return controller.loadMotionFile(controller.up_, nh, "/motionFilePathUp", "Up");
  }
  static bool warmup(AcController& controller, ActionPacket& output)
  {
    ObservationPacket input;
    input.policy = PolicyKind::UP;
    input.size = controller.up_.observations.size();
    input.data.assign(input.size, 0.0f);
    return controller.runUpPolicy(input, output);
  }
  static bool install() noexcept
  {
    return AcController::registerPolicyMode(
        {PolicyKind::UP, "UP", &run, &actions, &lastActions, &enter,
         nullptr, &loadMotion, &warmup, nullptr, true});
  }
};

namespace
{
[[maybe_unused]] const bool kUpModeRegistered =
    PolicyModeRegistrationBuilder<PolicyKind::UP>::install();
}
}  // namespace legged
