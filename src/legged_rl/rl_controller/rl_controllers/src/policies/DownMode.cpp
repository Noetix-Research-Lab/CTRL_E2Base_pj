#include "rl_controllers/AcController.h"

#include <algorithm>

namespace legged
{
void AcController::handleDownMode()
{
  // Completion leaves no policy active while ControlMode remains DOWN. A new
  // DOWN request can only arrive through WALK, in which case WALK is still
  // the active policy here and handleMotionMode() must be allowed to run its
  // DOWN entry hook and clear the completion latch.
  if (down_.dampingAfterCompletion && activePolicy_ == PolicyKind::NONE) {
    handleDownCompletionDamping();
    return;
  }
  handleMotionMode(PolicyKind::DOWN, down_,
                   ControlEvent::DOWN_COMPLETE, ControlEvent::DOWN_POLICY_ERROR,
                   ControlEvent::DOWN_ACTION_TIMEOUT, ControlEvent::DOWN_PROPRIO_MISMATCH);
}

void AcController::handleDownCompletionDamping() noexcept
{
  if (hybridJointHandles_.size() > kMaxTransitionDof) {
    downTorqueFadeActive_ = false;
    ROS_ERROR_THROTTLE(1.0, "DOWN torque fade joint count exceeds fixed buffer");
    for (HybridJointHandle& joint : hybridJointHandles_) {
      joint.setCommand(0, 0, 0, kDownCompletionDamping, 0);
    }
    return;
  }
  if (!downTorqueFadeActive_) {
    for (HybridJointHandle& joint : hybridJointHandles_) {
      joint.setCommand(0, 0, 0, kDownCompletionDamping, 0);
    }
    return;
  }

  const int64_t nowNs = steadyNowNs();
  const int64_t elapsedNs = nowNs >= downTorqueFadeStartNs_
      ? nowNs - downTorqueFadeStartNs_ : 0;
  const double linearBlend = downTorqueFadeDurationNs_ <= 0
      ? 1.0
      : std::clamp(static_cast<double>(elapsedNs) /
                       static_cast<double>(downTorqueFadeDurationNs_),
                   0.0, 1.0);
  const double blend = linearBlend * linearBlend * (3.0 - 2.0 * linearBlend);
  const double authority = 1.0 - blend;
  for (size_t index = 0; index < hybridJointHandles_.size(); ++index) {
    hybridJointHandles_[index].setCommand(
        downFadePositionDesired_[index],
        downFadeVelocityDesired_[index] * authority,
        downFadeKp_[index] * authority,
        downFadeKd_[index] * authority + kDownCompletionDamping * blend,
        downFadeFeedforward_[index] * authority);
  }
  if (linearBlend >= 1.0) {
    downTorqueFadeActive_ = false;
    ROS_INFO("DOWN torque fade complete: entering low-damping hold");
  }
}

void AcController::beginDownCompletionDamping() noexcept
{
  if (hybridJointHandles_.size() > kMaxTransitionDof) {
    downTorqueFadeActive_ = false;
    ROS_ERROR("DOWN torque fade joint count exceeds fixed buffer");
    return;
  }
  for (size_t index = 0; index < hybridJointHandles_.size(); ++index) {
    HybridJointHandle& joint = hybridJointHandles_[index];
    downFadePositionDesired_[index] = joint.getPositionDesired();
    downFadeVelocityDesired_[index] = joint.getVelocityDesired();
    downFadeKp_[index] = joint.getKp();
    downFadeKd_[index] = joint.getKd();
    downFadeFeedforward_[index] = joint.getFeedforward();
  }
  downTorqueFadeStartNs_ = steadyNowNs();
  downTorqueFadeActive_ = true;
  ROS_INFO("DOWN complete: fading policy torque into low-damping hold");
}

bool AcController::runDownPolicy(const ObservationPacket& input, ActionPacket& output)
{
  return runMotionPolicy(down_, ControlEvent::DOWN_POLICY_ERROR, input, output);
}

void AcController::resetDownPlaybackClock() noexcept
{
  downTorqueFadeActive_ = false;
  resetMotionPlaybackClock(down_);
}

template<>
struct PolicyModeRegistrationBuilder<PolicyKind::DOWN>
{
  static bool run(AcController& controller, const ObservationPacket& input,
                  ActionPacket& output)
  {
    return controller.runDownPolicy(input, output);
  }
  static std::vector<float>& actions(AcController& controller) { return controller.down_.actions; }
  static vector_t& lastActions(AcController& controller) { return controller.down_.lastActions; }
  static void enter(AcController& controller) { controller.resetDownPlaybackClock(); }
  static bool loadMotion(AcController& controller, ros::NodeHandle& nh)
  {
    return controller.loadMotionFile(controller.down_, nh, "/motionFilePathDown", "Down");
  }
  static bool warmup(AcController& controller, ActionPacket& output)
  {
    ObservationPacket input;
    input.policy = PolicyKind::DOWN;
    input.size = controller.down_.observations.size();
    input.data.assign(input.size, 0.0f);
    return controller.runDownPolicy(input, output);
  }
  static bool install() noexcept
  {
    return AcController::registerPolicyMode(
        {PolicyKind::DOWN, "DOWN", &run, &actions, &lastActions, &enter,
         nullptr, &loadMotion, &warmup, nullptr, true});
  }
};

namespace
{
[[maybe_unused]] const bool kDownModeRegistered =
    PolicyModeRegistrationBuilder<PolicyKind::DOWN>::install();
}
}  // namespace legged
