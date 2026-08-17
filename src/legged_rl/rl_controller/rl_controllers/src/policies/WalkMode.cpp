#include "rl_controllers/AcController.h"
#include "rl_controllers/RotationTools.h"

#include <algorithm>
#include <cmath>

namespace legged
{
void AcController::handleWalkMode()
{
  if (csvTrajectoryPlayer_.enabled())
  {
    leavePolicyMode();
    if (!csvPlaybackSessionActive_)
    {
      csvTrajectoryPlayer_.reset();
      csvDiagnosticLogger_.resetForNextRun();
      csv_log_start_pending_ = csvDiagnosticLogger_.loggingEnabled();
      csvPlaybackSessionActive_ = true;
    }
    playCsvTrajectory();
    return;
  }
  csvPlaybackSessionActive_ = false;
  enterPolicyMode(PolicyKind::WALK);
  auto& runtime = walkPolicy_.runtime();
  // if (propri_.projectedGravity(2) >= -0.3) {
  //   enqueueControlEvent(ControlEvent::FALL_PROTECTION);
  //   leavePolicyMode();
  //   mode_ = Mode::DEFAULT;
  //   RLControllerBase::handleDefaultMode();
  //   return;
  // }

  uint64_t policyTick = 0;
  if (runtime.playbackClock.take(runtime.policyPeriodNs, steadyNowNs(), policyTick)) {
    if (!computeWalkObservation()) {
      leavePolicyMode();
      mode_ = Mode::DEFAULT;
      RLControllerBase::handleDefaultMode();
      return;
    }
    if (asyncInferenceEnabled_) {
      submitObservation(PolicyKind::WALK, policyObservations_);
    } else {
      runSynchronousPolicy(PolicyKind::WALK, policyObservations_);
    }
  }
  if (asyncInferenceEnabled_) consumeAction(PolicyKind::WALK);

  if (!std::all_of(actions_.begin(), actions_.end(),
                   [](tensor_element_t action) {
                     return std::isfinite(action);
                   })) {
    enqueueControlEvent(ControlEvent::WALK_POLICY_ERROR);
    leavePolicyMode();
    mode_ = Mode::DEFAULT;
    RLControllerBase::handleDefaultMode();
    return;
  }
  const scalar_t actionMin = -robotCfg_.clipActions;
  const scalar_t actionMax = robotCfg_.clipActions;
  std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                 [actionMin, actionMax](scalar_t value) {
                   return std::max(actionMin, std::min(actionMax, value));
                 });
  if (actionTimedOut()) {
    enqueueControlEvent(ControlEvent::WALK_ACTION_TIMEOUT);
    leavePolicyMode();
    mode_ = Mode::DEFAULT;
    RLControllerBase::handleDefaultMode();
    return;
  }
  for (int i = 0; i < actionsSize_; ++i) {
    const scalar_t desired = actions_[i] * walkPolicy_.metadata().actionScale()[i] +
                             defaultJointAngles_(i);
    policyJointHandles_[i].setCommand(
        desired, 0, walkPolicy_.metadata().stiffness()[i],
        walkPolicy_.metadata().damping()[i], 0);
    lastActions_(i, 0) = actions_[i];
  }
}

bool AcController::runWalkPolicy(const ObservationPacket& input,
                                 ActionPacket& output)
{
  const PolicyInputView policyInput{input.data.data(), input.size};
  PolicyOutputView policyOutput;
  policyOutput.actions = output.actions.data();
  policyOutput.actionCapacity = output.actions.size();
  const PolicyRunStatus status =
      walkPolicy_.run(policyInput, policyOutput, inferenceRunOptions_);
  output.size = policyOutput.actionSize;
  output.estimatedVelocity = policyOutput.estimatedVelocity;
  output.estimateValid = policyOutput.estimateValid;
  if (status == PolicyRunStatus::SUCCESS) return true;
  // Velocity estimation is optional. Models with a projected-gravity input
  // may legitimately expose only the actions output.
  if (status == PolicyRunStatus::SUCCESS_WITHOUT_ESTIMATE) return true;
  enqueueControlEvent(ControlEvent::WALK_POLICY_ERROR);
  return false;
}

void AcController::computeObservation()
{
  computeWalkObservation();
}

bool AcController::computeWalkObservation()
{
  vector3_t command;
  if (std::abs(command_.x) < 0.3) command_.x = 0.0;
  if (std::abs(command_.y) < 0.3) command_.y = 0.0;
  if (std::abs(command_.yaw) < 0.3) command_.yaw = 0.0;
  if (std::abs(command_.y) > 0.3) {
    command_.x = 0.0;
    command_.yaw = 0.0;
  }
  if (std::abs(command_.x) > 0.3 && std::abs(command_.yaw) > 0.3)
    command_.y = 0.0;
  command << command_.x, command_.y, command_.yaw;

  vector_t& proprioObs = walkProprioObs_;
  if (policyPropri_.jointPos.size() != actionsSize_ ||
      policyPropri_.jointVel.size() != actionsSize_) {
    enqueueControlEvent(ControlEvent::WALK_PROPRIO_MISMATCH);
    return false;
  }
  if (walkUsesProjectedGravity_) {
    proprioObs << command, propri_.baseAngVel, propri_.projectedGravity,
        (policyPropri_.jointPos - defaultJointAngles_), policyPropri_.jointVel,
        lastActions_;
  } else {
    const vector3_t eulerAngle = quatToXyz(propri_.robot_quat_);
    proprioObs << command, propri_.baseAngVel, eulerAngle(0), eulerAngle(1),
        (policyPropri_.jointPos - defaultJointAngles_), policyPropri_.jointVel,
        lastActions_;
  }

  if (isfirstRecObs_) {
    for (int i = observationSize_ - actionsSize_; i < observationSize_; ++i)
      proprioObs(i, 0) = 0.0;
    for (int i = 0; i < stackSize_; ++i) {
      proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) =
          proprioObs.cast<tensor_element_t>();
    }
    isfirstRecObs_ = false;
    std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
  }
  proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
      proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
  proprioHistoryBuffer_.tail(observationSize_) = proprioObs.cast<tensor_element_t>();
  for (size_t i = 0; i < policyObservations_.size(); ++i)
    policyObservations_[i] = proprioHistoryBuffer_[static_cast<Eigen::Index>(i)];

  const scalar_t observationMin = -robotCfg_.clipObs;
  const scalar_t observationMax = robotCfg_.clipObs;
  std::transform(policyObservations_.begin(), policyObservations_.end(),
                 policyObservations_.begin(),
                 [observationMin, observationMax](scalar_t value) {
                   return std::max(observationMin,
                                   std::min(observationMax, value));
                 });
  return true;
}

void AcController::resetWalkPlaybackClock() noexcept
{
  const int64_t nowNs = steadyNowNs();
  walkPolicy_.runtime().playbackClock.reset(nowNs);
}

template<>
struct PolicyModeRegistrationBuilder<PolicyKind::WALK>
{
  static bool run(AcController& controller, const ObservationPacket& input,
                  ActionPacket& output)
  {
    return controller.runWalkPolicy(input, output);
  }
  static std::vector<float>& actions(AcController& controller) { return controller.actions_; }
  static vector_t& lastActions(AcController& controller) { return controller.lastActions_; }
  static void enter(AcController& controller) { controller.resetWalkPlaybackClock(); }
  static bool warmup(AcController& controller, ActionPacket& output)
  {
    ObservationPacket input;
    input.policy = PolicyKind::WALK;
    input.size = controller.policyObservations_.size();
    input.data.assign(input.size, 0.0f);
    return controller.runWalkPolicy(input, output);
  }
  static void publishTrace(AcController& controller, const ActionPacket& output)
  {
    if (!output.estimateValid ||
        controller.velEstimatePublisher_.getNumSubscribers() == 0) return;
    std_msgs::Float64MultiArray message;
    message.data.resize(3);
    for (size_t i = 0; i < 3; ++i) message.data[i] = output.estimatedVelocity[i];
    controller.velEstimatePublisher_.publish(message);
  }
  static bool install() noexcept
  {
    return AcController::registerPolicyMode(
        {PolicyKind::WALK, "WALK", &run, &actions, &lastActions, &enter,
         nullptr, nullptr, &warmup, &publishTrace, false});
  }
};

namespace
{
[[maybe_unused]] const bool kWalkModeRegistered =
    PolicyModeRegistrationBuilder<PolicyKind::WALK>::install();
}
}  // namespace legged
