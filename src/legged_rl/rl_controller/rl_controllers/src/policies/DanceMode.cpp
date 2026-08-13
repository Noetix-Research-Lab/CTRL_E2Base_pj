#include "rl_controllers/AcController.h"
#include "rl_controllers/RotationTools.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>

#include "nlohmann/json.hpp"

namespace
{
bool checkExactSize(const char* modeName, const std::string& label,
                    size_t actual, size_t expected)
{
  if (actual == expected) return true;
  ROS_ERROR_STREAM("[" << modeName << "Mode] " << label << " size mismatch: actual="
                   << actual << " expected=" << expected);
  return false;
}

bool loadMotionFile(legged::DancePolicy& policy, int actionSize,
                    ros::NodeHandle& nh, const char* parameterName,
                    const char* modeName)
{
  auto& runtime = policy.runtime();
  std::string motionFilePath;
  if (!nh.getParam(parameterName, motionFilePath)) {
    ROS_ERROR("[%sMode] Get motion path failed: %s", modeName, parameterName);
    return false;
  }
  std::ifstream file(motionFilePath);
  if (!file.is_open()) {
    ROS_ERROR("[%sMode] Cannot open motion file: %s", modeName, motionFilePath.c_str());
    return false;
  }

  nlohmann::json motion;
  try {
    file >> motion;
    runtime.jointPositionFrames =
        motion.at("joint_pos").get<std::vector<std::vector<double>>>();
    runtime.jointVelocityFrames =
        motion.at("joint_vel").get<std::vector<std::vector<double>>>();
    runtime.bodyQuaternionFrames =
        motion.at("body_quat_w").get<std::vector<std::vector<double>>>();
  } catch (const std::exception& exception) {
    ROS_ERROR("[%sMode] Invalid motion file '%s': %s", modeName,
              motionFilePath.c_str(), exception.what());
    return false;
  }
  runtime.timestepCount = runtime.jointPositionFrames.size();
  runtime.jointCount = runtime.timestepCount > 0
      ? runtime.jointPositionFrames[0].size() : 0;
  ROS_INFO("[%sMode] Loaded %zu motion frames with %zu joints", modeName,
           runtime.timestepCount, runtime.jointCount);

  if (runtime.timestepCount == 0) {
    ROS_ERROR("[%sMode] Motion file has no frames: %s", modeName, motionFilePath.c_str());
    return false;
  }
  if (!checkExactSize(modeName, "joint_vel frames", runtime.jointVelocityFrames.size(),
                      runtime.timestepCount) ||
      !checkExactSize(modeName, "body_quat_w frames", runtime.bodyQuaternionFrames.size(),
                      runtime.timestepCount)) {
    return false;
  }
  if (motion.contains("metadata")) {
    const auto& metadata = motion["metadata"];
    try {
      if (metadata.contains("dof") && metadata["dof"].get<int>() != actionSize) {
        ROS_ERROR_STREAM("[" << modeName << "Mode] Motion metadata.dof="
                         << metadata["dof"].get<int>()
                         << " does not match policy dof=" << actionSize);
        return false;
      }
      if (metadata.contains("joints_count") &&
          metadata["joints_count"].get<int>() != actionSize) {
        ROS_ERROR_STREAM("[" << modeName << "Mode] Motion metadata.joints_count="
                         << metadata["joints_count"].get<int>()
                         << " does not match policy dof=" << actionSize);
        return false;
      }
    } catch (const std::exception& exception) {
      ROS_ERROR("[%sMode] Invalid metadata in '%s': %s", modeName,
                motionFilePath.c_str(), exception.what());
      return false;
    }
  }
  for (size_t i = 0; i < runtime.timestepCount; ++i) {
    if (!checkExactSize(modeName, "joint_pos frame " + std::to_string(i),
                        runtime.jointPositionFrames[i].size(), actionSize) ||
        !checkExactSize(modeName, "joint_vel frame " + std::to_string(i),
                        runtime.jointVelocityFrames[i].size(), actionSize) ||
        !checkExactSize(modeName, "body_quat_w frame " + std::to_string(i),
                        runtime.bodyQuaternionFrames[i].size(), 4)) {
      return false;
    }
    for (double value : runtime.jointPositionFrames[i]) {
      if (!std::isfinite(value)) {
        ROS_ERROR("[%sMode] joint_pos frame %zu contains non-finite data", modeName, i);
        return false;
      }
    }
    for (double value : runtime.jointVelocityFrames[i]) {
      if (!std::isfinite(value)) {
        ROS_ERROR("[%sMode] joint_vel frame %zu contains non-finite data", modeName, i);
        return false;
      }
    }
    double quaternionNormSquared = 0.0;
    for (double value : runtime.bodyQuaternionFrames[i]) {
      if (!std::isfinite(value)) {
        ROS_ERROR("[%sMode] quaternion frame %zu contains non-finite data", modeName, i);
        return false;
      }
      quaternionNormSquared += value * value;
    }
    if (!std::isfinite(quaternionNormSquared) || quaternionNormSquared <= 1.0e-12) {
      ROS_ERROR("[%sMode] quaternion frame %zu is invalid", modeName, i);
      return false;
    }
  }
  return true;
}
}  // namespace

namespace legged
{
  void AcController::handleDanceMode()
  {
    handleMotionMode(PolicyKind::DANCE, dance_,
                     ControlEvent::DANCE_COMPLETE, ControlEvent::DANCE_POLICY_ERROR,
                     ControlEvent::DANCE_ACTION_TIMEOUT, ControlEvent::DANCE_PROPRIO_MISMATCH);
  }

  void AcController::handleMotionMode(
      PolicyKind kind, MotionPolicyState& state,
      ControlEvent completeEvent, ControlEvent policyErrorEvent,
      ControlEvent timeoutEvent, ControlEvent proprioErrorEvent)
  {
    enterPolicyMode(kind);
    auto& runtime = state.policy.runtime();
    const int64_t nowNs = steadyNowNs();
    const bool freezeUpMotion = kind == PolicyKind::UP && upPolicyBlendActive_;
    if (runtime.endPending && runtime.finalObservationSequence != 0 &&
        nowNs >= runtime.completionNotBeforeNs &&
        lastAppliedSequence_ >= runtime.finalObservationSequence) {
      enqueueControlEvent(completeEvent);
      isfirstRecObs_ = true;
      leavePolicyMode();
      if (kind == PolicyKind::DOWN) {
        // Remain in DOWN but stop policy inference after the final action has
        // been applied. Fade the final policy command into the low-damping
        // hold before removing position-control authority.
        state.dampingAfterCompletion = true;
        beginDownCompletionDamping();
        handleDownCompletionDamping();
      } else {
        mode_ = Mode::WALK;
      }
      return;
    }

    uint64_t policyTick = 0;
    if (!runtime.endPending &&
        runtime.playbackClock.take(runtime.policyPeriodNs, nowNs, policyTick)) {
      // Keep the first policy input exactly on JSON frame zero. Later inputs
      // use true elapsed time so control-loop jitter changes only the blend,
      // never the long-term playback phase.
      const double elapsedSec = policyTick == 0 ? 0.0 : static_cast<double>(
          std::max<int64_t>(0, nowNs - runtime.playbackClock.startNs())) / 1.0e9;
      const double durationSec = runtime.timestepCount <= 1 ? 0.0 :
          static_cast<double>(runtime.timestepCount - 1) / runtime.motionFrequencyHz;
      // While UP hands command authority to the network, repeatedly infer at
      // frame zero. Advancing the reference at the same time would make the
      // policy chase a moving target and reintroduce a jump at blend end.
      const double motionTimeSec = freezeUpMotion ? 0.0 : std::min(elapsedSec, durationSec);
      if (!computeObservationMotion(state, motionTimeSec, proprioErrorEvent,
                                    policyKindName(kind))) {
        leavePolicyMode();
        mode_ = Mode::DEFAULT;
        RLControllerBase::handleDefaultMode();
        return;
      }
      if (asyncInferenceEnabled_) {
        submitObservation(kind, state.observations);
      } else {
        runSynchronousPolicy(kind, state.observations);
      }
      if (!freezeUpMotion && elapsedSec >= durationSec) {
        runtime.endPending = true;
        runtime.finalObservationSequence = observationSequence_;
        runtime.completionNotBeforeNs = nowNs <=
            std::numeric_limits<int64_t>::max() - runtime.policyPeriodNs
            ? nowNs + runtime.policyPeriodNs
            : std::numeric_limits<int64_t>::max();
      }
    }
    if (asyncInferenceEnabled_) consumeAction(kind);
    if (!std::all_of(state.actions.begin(), state.actions.end(),
                     [](tensor_element_t action) {
                       return std::isfinite(action);
                     })) {
      enqueueControlEvent(policyErrorEvent);
      leavePolicyMode();
      mode_ = Mode::DEFAULT;
      RLControllerBase::handleDefaultMode();
      return;
    }
    scalar_t actionMin = -robotCfg_.clipActions;
    scalar_t actionMax =  robotCfg_.clipActions;
    std::transform(state.actions.begin(), state.actions.end(), state.actions.begin(),
                   [actionMin, actionMax](scalar_t x){ return std::max(actionMin, std::min(actionMax, x)); });

    if (actionTimedOut()) {
      enqueueControlEvent(timeoutEvent);
      leavePolicyMode();
      mode_ = Mode::DEFAULT;
      RLControllerBase::handleDefaultMode();
      return;
    }
    double upPolicyBlend = 1.0;
    if (freezeUpMotion) {
      const int64_t elapsedNs = nowNs >= upPolicyBlendStartNs_
          ? nowNs - upPolicyBlendStartNs_ : 0;
      const double linearBlend = upPolicyBlendDurationNs_ <= 0
          ? 1.0
          : std::clamp(static_cast<double>(elapsedNs) /
                           static_cast<double>(upPolicyBlendDurationNs_),
                       0.0, 1.0);
      // Smoothstep keeps desired-position velocity continuous at both ends.
      upPolicyBlend = linearBlend * linearBlend * (3.0 - 2.0 * linearBlend);
    }
    for (int i = 0; i < actionsSizeDance_; i++) {
      const scalar_t appliedAction = freezeUpMotion
          ? upPolicyBlendStartActions_(i) * (1.0 - upPolicyBlend) +
                state.actions[i] * upPolicyBlend
          : state.actions[i];
      scalar_t pos_des = appliedAction * state.policy.metadata().actionScale()[i] + state.defaultJointAngles(i);
      policyJointHandles_[i].setCommand(pos_des, 0, state.policy.metadata().stiffness()[i], state.policy.metadata().damping()[i], 0);
      state.lastActions(i, 0) = appliedAction;
    }
    if (freezeUpMotion && upPolicyBlend >= 1.0) {
      upPolicyBlendActive_ = false;
      resetMotionPlaybackClock(up_);
      ROS_INFO("UP policy blend complete: starting motion playback from frame 0");
    }
  }


  bool AcController::runDancePolicy(const ObservationPacket& input, ActionPacket& output)
  {
    return runMotionPolicy(dance_, ControlEvent::DANCE_POLICY_ERROR, input, output);
  }

  bool AcController::runMotionPolicy(MotionPolicyState& state,
                                     ControlEvent policyErrorEvent,
                                     const ObservationPacket& input,
                                     ActionPacket& output)
  {
    const PolicyInputView policyInput{input.data.data(), input.size};
    PolicyOutputView policyOutput;
    policyOutput.actions = output.actions.data();
    policyOutput.actionCapacity = output.actions.size();
    const PolicyRunStatus status =
        state.policy.run(policyInput, policyOutput, inferenceRunOptions_);
    output.size = policyOutput.actionSize;
    if (status != PolicyRunStatus::SUCCESS) {
      enqueueControlEvent(policyErrorEvent);
      return false;
    }
    return true;
  }


  void AcController::transformBaseOriToTorsoOri(const Eigen::Quaterniond& base_quat,
                                                  const std::array<double, 3>& waist_joint_angles,
                                                  Eigen::Quaterniond& torso_quat)
  {
    Eigen::AngleAxisd rollAngle1(0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle1(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle1(waist_joint_angles[0], Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d transform_rot_mat_1 = (yawAngle1 * pitchAngle1 * rollAngle1).toRotationMatrix();

    Eigen::AngleAxisd rollAngle2(waist_joint_angles[1], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle2(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle2(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d transform_rot_mat_2 = (yawAngle2 * pitchAngle2 * rollAngle2).toRotationMatrix();

    Eigen::AngleAxisd pitchAngle3(waist_joint_angles[2], Eigen::Vector3d::UnitY());
    Eigen::Matrix3d transform_rot_mat =
        transform_rot_mat_1 * transform_rot_mat_2 * pitchAngle3.toRotationMatrix();
    Eigen::Matrix3d torso_rot_mat = base_quat.toRotationMatrix() * transform_rot_mat;
    torso_quat = matrix_to_quaternion_eigen<double>(torso_rot_mat);
  }


  void AcController::resetDancePlaybackClock() noexcept
  {
    resetMotionPlaybackClock(dance_);
  }

  void AcController::resetMotionPlaybackClock(MotionPolicyState& state) noexcept
  {
    auto& runtime = state.policy.runtime();
    const int64_t nowNs = steadyNowNs();
    runtime.playbackClock.reset(nowNs);
    runtime.endPending = false;
    runtime.finalObservationSequence = 0;
    runtime.completionNotBeforeNs = 0;
    runtime.alignmentSamples = 0;
    state.timeStep = 0;
    state.dampingAfterCompletion = false;
  }


  bool AcController::computeObservationDance(double motionTimeSec)
  {
    return computeObservationMotion(dance_, motionTimeSec,
                                    ControlEvent::DANCE_PROPRIO_MISMATCH,
                                    "DANCE");
  }

  bool AcController::loadMotionFile(MotionPolicyState& state, ros::NodeHandle& nh,
                                    const char* parameterName, const char* modeName)
  {
    return ::loadMotionFile(state.policy, actionsSizeDance_, nh, parameterName, modeName);
  }

  bool AcController::computeObservationMotion(MotionPolicyState& state,
                                               double motionTimeSec,
                                               ControlEvent proprioErrorEvent,
                                               const char* /*modeName*/)
  {
    auto& runtime = state.policy.runtime();
    if (runtime.timestepCount == 0 || runtime.motionFrequencyHz <= 0.0 ||
        runtime.referenceJointPosition.size() != actionsSizeDance_ ||
        runtime.referenceJointVelocity.size() != actionsSizeDance_ ||
        policyPropri_.jointPos.size() != actionsSizeDance_ ||
        policyPropri_.jointVel.size() != actionsSizeDance_) {
      enqueueControlEvent(proprioErrorEvent);
      return false;
    }

    const double continuousFrame = std::clamp(
        motionTimeSec * runtime.motionFrequencyHz, 0.0,
        static_cast<double>(runtime.timestepCount - 1));
    const size_t frame0 = static_cast<size_t>(std::floor(continuousFrame));
    const size_t frame1 = std::min(frame0 + 1, runtime.timestepCount - 1);
    const double blend = continuousFrame - static_cast<double>(frame0);
    state.timeStep = static_cast<int>(frame0);

    if (runtime.jointPositionFrames[frame0].size() != static_cast<size_t>(actionsSizeDance_) ||
        runtime.jointPositionFrames[frame1].size() != static_cast<size_t>(actionsSizeDance_) ||
        runtime.jointVelocityFrames[frame0].size() != static_cast<size_t>(actionsSizeDance_) ||
        runtime.jointVelocityFrames[frame1].size() != static_cast<size_t>(actionsSizeDance_) ||
        runtime.bodyQuaternionFrames[frame0].size() != 4 ||
        runtime.bodyQuaternionFrames[frame1].size() != 4) {
      enqueueControlEvent(proprioErrorEvent);
      return false;
    }
    for (int joint = 0; joint < actionsSizeDance_; ++joint) {
      runtime.referenceJointPosition(joint) =
          runtime.jointPositionFrames[frame0][joint] * (1.0 - blend) +
          runtime.jointPositionFrames[frame1][joint] * blend;
      runtime.referenceJointVelocity(joint) =
          runtime.jointVelocityFrames[frame0][joint] * (1.0 - blend) +
          runtime.jointVelocityFrames[frame1][joint] * blend;
    }
    const auto& quat0 = runtime.bodyQuaternionFrames[frame0];
    const auto& quat1 = runtime.bodyQuaternionFrames[frame1];
    Eigen::Quaterniond referenceQuat0(quat0[0], quat0[1], quat0[2], quat0[3]);
    Eigen::Quaterniond referenceQuat1(quat1[0], quat1[1], quat1[2], quat1[3]);
    if (!std::isfinite(referenceQuat0.squaredNorm()) ||
        !std::isfinite(referenceQuat1.squaredNorm()) ||
        referenceQuat0.squaredNorm() <= 1.0e-12 ||
        referenceQuat1.squaredNorm() <= 1.0e-12) {
      enqueueControlEvent(proprioErrorEvent);
      return false;
    }
    referenceQuat0.normalize();
    referenceQuat1.normalize();
    const Eigen::Quaterniond ref_robot_quat = referenceQuat0.slerp(blend, referenceQuat1);

    auto getPolicyJointPos = [this](const std::string& name) -> double {
      auto it = std::find(policyJointNames_.begin(), policyJointNames_.end(), name);
      if (it == policyJointNames_.end()) return 0.0;
      const auto idx = std::distance(policyJointNames_.begin(), it);
      return policyPropri_.jointPos(idx);
    };
    const std::array<double, 3> waist_joint_angles{{
        getPolicyJointPos("waist_yaw_joint"),
        getPolicyJointPos("waist_roll_joint"),
        getPolicyJointPos("waist_pitch_joint")}};
    Eigen::Quaterniond torso_quat;
    transformBaseOriToTorsoOri(propri_.robot_quat_, waist_joint_angles, torso_quat);

    const auto& robot_quat = Eigen::Quaterniond(torso_quat.w(), torso_quat.x(),
                                                 torso_quat.y(), torso_quat.z());

    if (runtime.alignmentSamples < 2) {
      Eigen::Quaterniond yaw_motion_quat = yaw_quat<double>(ref_robot_quat);
      Eigen::Matrix3d yaw_motion_matrix  = yaw_motion_quat.toRotationMatrix();
      Eigen::Quaterniond yaw_robot_quat  = yaw_quat<double>(robot_quat);
      Eigen::Matrix3d yaw_robot_matrix   = yaw_robot_quat.toRotationMatrix();
      runtime.initialToWorld = matrix_to_quaternion_eigen<double>(
          yaw_robot_matrix * yaw_motion_matrix.transpose());
      ++runtime.alignmentSamples;
    }

    Eigen::Quaterniond delta_q = subtract_frame_transforms<double>(
        robot_quat, ref_robot_quat, runtime.initialToWorld);
    Eigen::Matrix3d    rotation_mat = QuatToMat<double>(delta_q);
    vector_t& proprioObs = state.proprioObs;
    Eigen::Matrix<scalar_t, 6, 1> motion_anchor_ori_b;
    if (policyPropri_.jointPos.size() != actionsSizeDance_ ||
        policyPropri_.jointVel.size() != actionsSizeDance_ ||
        runtime.referenceJointPosition.size() != actionsSizeDance_ ||
        runtime.referenceJointVelocity.size() != actionsSizeDance_) {
      enqueueControlEvent(proprioErrorEvent);
      return false;
    }
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 2; ++j)
        motion_anchor_ori_b(i * 2 + j) = rotation_mat(i, j);

    proprioObs << runtime.referenceJointPosition, runtime.referenceJointVelocity,
        motion_anchor_ori_b, propri_.baseAngVel,
        (policyPropri_.jointPos - state.defaultJointAngles), policyPropri_.jointVel, state.lastActions;

    if (isfirstRecObs_) {
      for (int i = observationSizeDance_ - actionsSizeDance_; i < observationSizeDance_; i++) proprioObs(i, 0) = 0.0;
      for (size_t i = 0; i < (size_t)stackSizeDance_; i++)
        state.historyBuffer.segment(i * observationSizeDance_, observationSizeDance_) = proprioObs.cast<tensor_element_t>();
      isfirstRecObs_ = false;
      std::fill(state.observations.begin(), state.observations.end(), 0.0f);
    }
    state.historyBuffer.head(state.historyBuffer.size() - observationSizeDance_) =
        state.historyBuffer.tail(state.historyBuffer.size() - observationSizeDance_);
    state.historyBuffer.tail(observationSizeDance_) = proprioObs.cast<tensor_element_t>();
    for (size_t i = 0; i < (size_t)(observationSizeDance_ * stackSizeDance_); i++)
      state.observations[i] = static_cast<tensor_element_t>(state.historyBuffer[i]);
    scalar_t obsMin = -robotCfg_.clipObs, obsMax = robotCfg_.clipObs;
    std::transform(state.observations.begin(), state.observations.end(), state.observations.begin(),
                   [obsMin, obsMax](scalar_t x){ return std::max(obsMin, std::min(obsMax, x)); });
    return true;
  }

  template<>
  struct PolicyModeRegistrationBuilder<PolicyKind::DANCE>
  {
    static bool run(AcController& controller, const ObservationPacket& input,
                    ActionPacket& output)
    {
      return controller.runDancePolicy(input, output);
    }
    static std::vector<float>& actions(AcController& controller)
    {
      return controller.dance_.actions;
    }
    static vector_t& lastActions(AcController& controller)
    {
      return controller.dance_.lastActions;
    }
    static void enter(AcController& controller) { controller.resetDancePlaybackClock(); }
    static bool loadMotion(AcController& controller, ros::NodeHandle& nh)
    {
      return controller.loadMotionFile(controller.dance_, nh,
                                       "/motionFilePath", "Dance");
    }
    static bool warmup(AcController& controller, ActionPacket& output)
    {
      ObservationPacket input;
      input.policy = PolicyKind::DANCE;
      input.size = controller.dance_.observations.size();
      input.data.assign(input.size, 0.0f);
      return controller.runDancePolicy(input, output);
    }
    static bool install() noexcept
    {
      return AcController::registerPolicyMode(
          {PolicyKind::DANCE, "DANCE", &run, &actions, &lastActions, &enter,
           nullptr, &loadMotion, &warmup, nullptr, true});
    }
  };

  namespace
  {
  [[maybe_unused]] const bool kDanceModeRegistered =
      PolicyModeRegistrationBuilder<PolicyKind::DANCE>::install();
  }

}  // namespace legged
