#include "rl_controllers/AcController.h"
#include "rl_controllers/PolicyContractValidator.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <type_traits>
#include <unistd.h>

namespace legged
{
  static_assert(std::is_abstract<RLControllerBase>::value,
                "RLControllerBase must remain abstract");
  static_assert(std::is_abstract<AcController>::value,
                "AcController must remain abstract");

  // Pluginlib needs an instantiable type. Keep that mechanical adapter private
  // so the public controller layers remain abstract and mode implementations
  // continue to live in their corresponding translation units.
  class AcControllerPlugin final : public AcController
  {
  protected:
    void handleWalkMode() override { AcController::handleWalkMode(); }
    void handleDanceMode() override { AcController::handleDanceMode(); }
    void handleDownMode() override { AcController::handleDownMode(); }
    void handleUpMode() override { AcController::handleUpMode(); }
    void handleDefaultMode() override { AcController::handleDefaultMode(); }
    void handleLieMode() override { AcController::handleLieMode(); }
    void handleStandMode() override { AcController::handleStandMode(); }
  };

  AcController::AcController()
    : policyModes_(policyModeCatalog())
  {
    for (const PolicyModeRegistration& mode : policyModes_) {
      if (mode.policy == PolicyKind::NONE || mode.runner == nullptr ||
          mode.actions == nullptr || mode.lastActions == nullptr ||
          mode.onEnter == nullptr || mode.warmup == nullptr) {
        throw std::logic_error("AcController policy mode registration is incomplete");
      }
    }
  }

  std::array<AcController::PolicyModeRegistration, AcController::kPolicyModeCount>&
  AcController::policyModeCatalog() noexcept
  {
    static std::array<PolicyModeRegistration, kPolicyModeCount> catalog{};
    return catalog;
  }

  bool AcController::registerPolicyMode(const PolicyModeRegistration& mode) noexcept
  {
    const size_t index = static_cast<size_t>(mode.policy);
    if (index == 0 || index > kPolicyModeCount) return false;
    PolicyModeRegistration& slot = policyModeCatalog()[index - 1];
    if (slot.policy != PolicyKind::NONE) return false;
    slot = mode;
    return true;
  }

  const AcController::PolicyModeRegistration* AcController::findPolicyMode(
      PolicyKind policy) const noexcept
  {
    const size_t index = static_cast<size_t>(policy);
    if (index == 0 || index > policyModes_.size()) return nullptr;
    const PolicyModeRegistration& mode = policyModes_[index - 1];
    return mode.policy == policy ? &mode : nullptr;
  }

  AcController::~AcController()
  {
    stopInferenceThread();
  }

  bool AcController::init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH)
  {
    if (!RLControllerBase::init(robotHw, controllerNH)) {
      return false;
    }

    initializeAsyncInference();
    inferenceRuntime_.terminationRequested.store(false, std::memory_order_release);
    inferenceRunOptions_.UnsetTerminate();
    if (!warmupPolicies()) {
      ROS_ERROR("[AcController] Policy warmup failed; refusing to start controller");
      return false;
    }
    // This non-RT worker owns timing publication and synchronous-policy trace
    // publication. It therefore runs even when timing metrics are disabled.
    inferenceRuntime_.timingPublisherRunning.store(true, std::memory_order_release);
    inferenceRuntime_.timingPublisherThread = std::thread(&AcController::timingPublisherLoop, this);
    if (asyncInferenceEnabled_) {
      inferenceRuntime_.setupStatus.store(0, std::memory_order_release);
      inferenceRuntime_.inferenceRunning.store(true, std::memory_order_release);
      inferenceRuntime_.inferenceThread = std::thread(&AcController::inferenceLoop, this);
      if (requireRealtime_) {
        const auto setupDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (inferenceRuntime_.setupStatus.load(std::memory_order_acquire) == 0 &&
               std::chrono::steady_clock::now() < setupDeadline) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (inferenceRuntime_.setupStatus.load(std::memory_order_acquire) != 1) {
          ROS_ERROR("[AcController] require_realtime: inference thread affinity/SCHED_FIFO setup failed");
          stopInferenceThread();
          return false;
        }
      }
    }
    // This also covers synchronous inference: on ROS shutdown the control-loop
    // owner may wait for update() before the controller destructor can run.
    inferenceRuntime_.shutdownWatcherRunning.store(true, std::memory_order_release);
    inferenceRuntime_.shutdownWatcherThread = std::thread(&AcController::shutdownWatcherLoop, this);
    return true;
  }

  void AcController::starting(const ros::Time &time)
  {
    RLControllerBase::starting(time);
    inferenceRuntime_.controllerActive.store(false, std::memory_order_release);
    inferenceRuntime_.policyEpoch.fetch_add(1, std::memory_order_acq_rel);
    activePolicy_ = PolicyKind::NONE;
    upPreparationActive_ = false;
    upGainsApplied_ = false;
    upTransitionStartNs_ = 0;
    upPolicyBlendActive_ = false;
    upPolicyBlendStartNs_ = 0;
    downTorqueFadeActive_ = false;
    downTorqueFadeStartNs_ = 0;
    observationSequence_ = 0;
    lastAppliedSequence_ = 0;
    policyEntryTimestampNs_ = 0;
    lastAppliedObservationTimestampNs_ = 0;
    for (const PolicyModeRegistration& mode : policyModes_) {
      std::vector<tensor_element_t>& actions = mode.actions(*this);
      std::fill(actions.begin(), actions.end(), 0.0f);
      mode.lastActions(*this).setZero();
      if (mode.onReset != nullptr) mode.onReset(*this);
    }
    inferenceRuntime_.controllerActive.store(true, std::memory_order_release);
    inferenceRuntime_.notifyInferenceStateChange();
  }

  void AcController::stopping(const ros::Time & /*time*/)
  {
    inferenceRuntime_.controllerActive.store(false, std::memory_order_release);
    inferenceRuntime_.notifyInferenceStateChange();
    if (tracksCumulativeInference(activePolicy_)) {
      endPolicyInferenceStats(activePolicy_,
                              inferenceRuntime_.policyEpoch.load(std::memory_order_acquire), steadyNowNs());
    }
    inferenceRuntime_.policyEpoch.fetch_add(1, std::memory_order_acq_rel);
    activePolicy_ = PolicyKind::NONE;
  }

  int64_t AcController::steadyNowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void AcController::publishTrace(const ros::Publisher& publisher,
                                  const std::vector<tensor_element_t>& data,
                                  size_t size) const
  {
    if (publisher.getNumSubscribers() == 0) return;
    std_msgs::Float64MultiArray msg;
    size = std::min(size, data.size());
    msg.data.reserve(size);
    for (size_t i = 0; i < size; ++i) msg.data.push_back(static_cast<double>(data[i]));
    publisher.publish(msg);
  }

  // ================================================================
  //  loadMotions
  // ================================================================
  bool AcController::loadMotions(ros::NodeHandle &nh)
  {
    for (const PolicyModeRegistration& mode : policyModes_) {
      if (mode.loadMotion != nullptr && !mode.loadMotion(*this, nh)) return false;
    }
    return true;
  }

  // ================================================================
  //  loadModel
  // ================================================================
  bool AcController::loadModel(ros::NodeHandle &nh)
  {
    std::string policyFilePath;
    std::string policyFilePathDance;
    std::string policyFilePathDown;
    std::string policyFilePathUp;
    if (!nh.getParam("/policyFile", policyFilePath)) {
      ROS_ERROR_STREAM("Get policy path fail!");
      return false;
    }
    ROS_INFO_STREAM("Load Onnx model from path : " << policyFilePath);

    if (!nh.getParam("/policyFileDance", policyFilePathDance)) {
      ROS_ERROR_STREAM("Get dance policy path fail!");
      return false;
    }
    ROS_INFO_STREAM("Load Onnx model from path : " << policyFilePathDance);
    if (!nh.getParam("/policyFileDown", policyFilePathDown)) {
      ROS_ERROR_STREAM("Get down policy path fail!");
      return false;
    }
    if (!nh.getParam("/policyFileUp", policyFilePathUp)) {
      ROS_ERROR_STREAM("Get up policy path fail!");
      return false;
    }
    ROS_INFO_STREAM("Load Onnx model from path : " << policyFilePathDown);
    ROS_INFO_STREAM("Load Onnx model from path : " << policyFilePathUp);

    onnxEnv_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(1);
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);

    walkPolicy_.load(*onnxEnv_, policyFilePath, sessionOptions);
    dance_.policy.load(*onnxEnv_, policyFilePathDance, sessionOptions);
    down_.policy.load(*onnxEnv_, policyFilePathDown, sessionOptions);
    up_.policy.load(*onnxEnv_, policyFilePathUp, sessionOptions);

    const bool foundJointNames = !walkPolicy_.metadata().jointNames().empty();
    const bool foundJointStiffness = !walkPolicy_.metadata().stiffness().empty();
    const bool foundJointDamping = !walkPolicy_.metadata().damping().empty();
    const bool foundDefaultJointPosition = !walkPolicy_.metadata().defaultPosition().empty();
    const bool foundActionScale = !walkPolicy_.metadata().actionScale().empty();

    if (!foundJointNames) ROS_ERROR("Missing metadata: 'joint_names'");
    if (!foundJointStiffness) ROS_ERROR("Missing metadata: 'joint_stiffness'");
    if (!foundJointDamping) ROS_ERROR("Missing metadata: 'joint_damping'");
    if (!foundDefaultJointPosition) ROS_ERROR("Missing metadata: 'default_joint_pos'");
    if (!foundActionScale) ROS_ERROR("Missing metadata: 'action_scale'");
    if (!foundJointNames || !foundJointStiffness || !foundJointDamping ||
        !foundDefaultJointPosition || !foundActionScale) {
      return false;
    }

    ROS_INFO_STREAM("Load Onnx model successfully !!!");
    return true;
  }

  // ================================================================
  //  loadRLCfg
  // ================================================================
  void AcController::initializePolicyRosInterfaces()
  {
    ros::NodeHandle rootNh;
    policyObservationPublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_policy_observation", 1);
    policyActionPublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_policy_action", 1);
    policyHistoryPublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_policy_observation_current", 1);
    policyCommandPublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_policy_observation_future", 1);
    velEstimatePublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/estimated_vel", 1);
    inferenceTimingPublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_inference_timing", 10);
    actionTimingPublisher_ =
        rootNh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_action_timing", 10);

  }

  bool AcController::loadPolicyRuntimeConfig(ros::NodeHandle& nh)
  {
    PolicyRuntimeConfig config;
    nh.param<int>("/LeggedRobotCfg/control/action_timeout_ms", config.actionTimeoutMs, 100);
    nh.param<int>("/LeggedRobotCfg/control/inference_cpu_affinity", config.inferenceCpuAffinity, -1);
    nh.param<int>("/LeggedRobotCfg/control/inference_thread_priority", config.inferenceThreadPriority, 20);
    nh.param<int>("/LeggedRobotCfg/control/warmup_runs", config.warmupRuns, 20);
    nh.param<bool>("/LeggedRobotCfg/control/async_inference", config.asyncInference, true);
    nh.param<bool>("/LeggedRobotCfg/control/enable_timing_metrics", config.enableTimingMetrics, false);
    nh.param<bool>("/LeggedRobotCfg/control/require_realtime", config.requireRealtime, false);
    nh.param<double>("/LeggedRobotCfg/control/walk_policy_frequency",
                     config.walkPolicyFrequencyHz, 50.0);
    nh.param<double>("/LeggedRobotCfg/control/dance_policy_frequency",
                     config.dancePolicyFrequencyHz, 50.0);
    nh.param<double>("/LeggedRobotCfg/control/dance_motion_frequency",
                     config.danceMotionFrequencyHz, 50.0);
    nh.param<double>("/LeggedRobotCfg/control/up_transition_duration",
                     config.upTransitionDurationSec, 2.0);
    nh.param<double>("/LeggedRobotCfg/control/up_policy_blend_duration",
                     config.upPolicyBlendDurationSec, 0.5);
    nh.param<double>("/LeggedRobotCfg/control/down_torque_fade_duration",
                     config.downTorqueFadeDurationSec, 1.0);

    std::string error;
    if (!config.validateAndNormalize(error)) {
      ROS_ERROR("[AcController] Invalid policy runtime config: %s", error.c_str());
      return false;
    }
    actionTimeoutMs_ = config.actionTimeoutMs;
    inferenceCpuAffinity_ = config.inferenceCpuAffinity;
    inferenceThreadPriority_ = config.inferenceThreadPriority;
    warmupRuns_ = config.warmupRuns;
    upTransitionDurationNs_ = config.upTransitionDurationNs;
    upPolicyBlendDurationNs_ = config.upPolicyBlendDurationNs;
    downTorqueFadeDurationNs_ = config.downTorqueFadeDurationNs;
    asyncInferenceEnabled_ = config.asyncInference;
    enableTimingMetrics_ = config.enableTimingMetrics;
    requireRealtime_ = config.requireRealtime;
    walkPolicy_.runtime().policyPeriodNs = config.walkPolicyPeriodNs;
    dance_.policy.runtime().policyPeriodNs = config.dancePolicyPeriodNs;
    dance_.policy.runtime().motionFrequencyHz = config.danceMotionFrequencyHz;
    down_.policy.runtime().policyPeriodNs = config.dancePolicyPeriodNs;
    down_.policy.runtime().motionFrequencyHz = config.danceMotionFrequencyHz;
    up_.policy.runtime().policyPeriodNs = config.dancePolicyPeriodNs;
    up_.policy.runtime().motionFrequencyHz = config.danceMotionFrequencyHz;
    return true;
  }

  void AcController::initializePolicyState()
  {
    actions_.assign(actionsSize_, 0.0F);
    policyObservations_.assign(observationSize_ * stackSize_, 0.0F);
    command_.x = 0.0;
    command_.y = 0.0;
    command_.yaw = 0.0;
    lastActions_.setZero(actionsSize_);
    proprioHistoryBuffer_.setZero(observationSize_ * stackSize_);
    walkProprioObs_.setZero(observationSize_);

    defaultJointAngles_.resize(actionsSize_);
    for (int index = 0; index < actionsSize_; ++index) {
      defaultJointAngles_(index) = walkPolicy_.metadata().defaultPosition()[index];
    }
    for (MotionPolicyState* state : {&dance_, &down_, &up_}) {
      state->actions.assign(actionsSizeDance_, 0.0F);
      state->observations.assign(observationSizeDance_ * stackSizeDance_, 0.0F);
      state->lastActions.setZero(actionsSizeDance_);
      state->historyBuffer.setZero(observationSizeDance_ * stackSizeDance_);
      state->proprioObs.setZero(observationSizeDance_);
      state->defaultJointAngles.resize(actionsSizeDance_);
      for (int index = 0; index < actionsSizeDance_; ++index) {
        state->defaultJointAngles(index) = state->policy.metadata().defaultPosition()[index];
      }
    }
    upTransitionStartAngles_.setZero(actionsSizeDance_);
    upPolicyBlendStartActions_.setZero(actionsSizeDance_);
  }

  bool AcController::loadRLCfg(ros::NodeHandle &nh)
  {
    initializePolicyRosInterfaces();

    int error = 0;

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations",robotCfg_.clipObs));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions",     robotCfg_.clipActions));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size",         actionsSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size_dance",   actionsSizeDance_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size",    observationSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_dance_size", observationSizeDance_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/stack_size",           stackSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/stack_size_dance",     stackSizeDance_));
    if (!loadPolicyRuntimeConfig(nh)) return false;
    if (error != 0) {
      ROS_ERROR_STREAM("[AcController] Missing required RL config entries, error count=" << error);
      return false;
    }
    if (actionsSize_ != policyDofNum_ || actionsSizeDance_ != policyDofNum_) {
      ROS_ERROR_STREAM("[AcController] actions_size/actions_size_dance must match policy dof. "
                       << "actions=" << actionsSize_ << " dance=" << actionsSizeDance_
                       << " policy_dof=" << policyDofNum_);
      return false;
    }
    const int expectedWalkObsEuler = 8 + 3 * actionsSize_;
    const int expectedWalkObsGravity = 9 + 3 * actionsSize_;
    const int expectedDanceObs = 9 + 5 * actionsSizeDance_;

    const auto& walkInputShapes = walkPolicy_.modelInfo().inputShapes();
    const int64_t walkInputElements = walkInputShapes.empty() ? -1 :
        PolicyContractValidator::elementCount(walkInputShapes[0]);
    if (walkInputElements > 0 && stackSize_ > 0 && walkInputElements % stackSize_ == 0) {
      const int modelWalkObs = static_cast<int>(walkInputElements / stackSize_);
      if (modelWalkObs == expectedWalkObsEuler || modelWalkObs == expectedWalkObsGravity) {
        if (observationSize_ != modelWalkObs) {
          ROS_WARN_STREAM("[AcController] Overriding walk observations_size from config "
                          << observationSize_ << " to ONNX-derived " << modelWalkObs);
        }
        observationSize_ = modelWalkObs;
      }
    }

    if ((observationSize_ != expectedWalkObsEuler && observationSize_ != expectedWalkObsGravity) ||
        observationSizeDance_ != expectedDanceObs) {
      ROS_ERROR_STREAM("[AcController] Observation size mismatch. walk=" << observationSize_
                       << " expected=" << expectedWalkObsEuler << " (euler_angles) or "
                       << expectedWalkObsGravity << " (projected_gravity), dance=" << observationSizeDance_
                       << " expected=" << expectedDanceObs);
      return false;
    }
    walkUsesProjectedGravity_ = (observationSize_ == expectedWalkObsGravity);
    walkPolicy_.configure(static_cast<size_t>(actionsSize_), walkUsesProjectedGravity_);
    dance_.policy.configure(static_cast<size_t>(actionsSizeDance_));
    down_.policy.configure(static_cast<size_t>(actionsSizeDance_));
    up_.policy.configure(static_cast<size_t>(actionsSizeDance_));
    ROS_INFO_STREAM("[AcController] Walk observation orientation mode: "
                    << (walkUsesProjectedGravity_ ? "projected_gravity" : "euler_angles"));

    std::string contractError;
    if (!PolicyContractValidator::metadata(
            walkPolicy_.metadata(), static_cast<size_t>(actionsSize_),
            policyJointNames_, contractError) ||
        !PolicyContractValidator::metadata(
            dance_.policy.metadata(), static_cast<size_t>(actionsSizeDance_),
            policyJointNames_, contractError) ||
        !PolicyContractValidator::metadata(
            down_.policy.metadata(), static_cast<size_t>(actionsSizeDance_),
            policyJointNames_, contractError) ||
        !PolicyContractValidator::metadata(
            up_.policy.metadata(), static_cast<size_t>(actionsSizeDance_),
            policyJointNames_, contractError)) {
      ROS_ERROR("[AcController] Policy metadata contract failed: %s", contractError.c_str());
      return false;
    }
    for (size_t j = 0; j < walkPolicy_.metadata().jointNames().size(); ++j) {
      ROS_INFO_STREAM("Joint: " << walkPolicy_.metadata().jointNames()[j]
        << " kp=" << walkPolicy_.metadata().stiffness()[j] << " kd=" << walkPolicy_.metadata().damping()[j]
        << " def=" << walkPolicy_.metadata().defaultPosition()[j] << " scale=" << walkPolicy_.metadata().actionScale()[j]);
    }

    initializePolicyState();

    int walkActionsShapeIdx = walkPolicy_.modelInfo().outputIndex("actions");
    if (walkActionsShapeIdx < 0) walkActionsShapeIdx = 0;
    int walkEstimateShapeIdx = walkPolicy_.modelInfo().outputIndex("estimate");
    if (!PolicyContractValidator::tensor(walkPolicy_.modelInfo(), true, 0,
                                         observationSize_ * stackSize_, "walk input", contractError) ||
        !PolicyContractValidator::tensor(walkPolicy_.modelInfo(), false,
                                         static_cast<size_t>(walkActionsShapeIdx), actionsSize_,
                                         "walk actions output", contractError) ||
        !PolicyContractValidator::tensor(dance_.policy.modelInfo(), true, 0,
                                         observationSizeDance_ * stackSizeDance_,
                                         "dance input", contractError) ||
        !PolicyContractValidator::tensor(dance_.policy.modelInfo(), false, 0,
                                         actionsSizeDance_, "dance output", contractError) ||
        !PolicyContractValidator::tensor(down_.policy.modelInfo(), true, 0,
                                         observationSizeDance_ * stackSizeDance_,
                                         "down input", contractError) ||
        !PolicyContractValidator::tensor(down_.policy.modelInfo(), false, 0,
                                         actionsSizeDance_, "down output", contractError) ||
        !PolicyContractValidator::tensor(up_.policy.modelInfo(), true, 0,
                                         observationSizeDance_ * stackSizeDance_,
                                         "up input", contractError) ||
        !PolicyContractValidator::tensor(up_.policy.modelInfo(), false, 0,
                                         actionsSizeDance_, "up output", contractError)) {
      ROS_ERROR("[AcController] Policy tensor contract failed: %s", contractError.c_str());
      return false;
    }
    if (walkUsesProjectedGravity_) {
      if (walkEstimateShapeIdx >= 0) {
        if (!PolicyContractValidator::tensor(
                walkPolicy_.modelInfo(), false, static_cast<size_t>(walkEstimateShapeIdx),
                3, "walk estimate output", contractError)) {
          ROS_ERROR("[AcController] Policy tensor contract failed: %s", contractError.c_str());
          return false;
        }
      } else {
        ROS_WARN_STREAM("[AcController] Walk policy uses projected_gravity but has no 'estimate' output.");
      }
    }

    buildControlJointMaps();
    if (!loadPdTestConfig(nh)) {
      return false;
    }
    return true;
  }

  bool AcController::controlRequestAllowed(ControlRequestId requestId) const noexcept
  {
    if (requestId != ControlRequestId::UP) return true;
    return mode_ == Mode::DOWN && down_.dampingAfterCompletion &&
           activePolicy_ == PolicyKind::NONE;
  }

  // ================================================================
  //  Asynchronous inference
  // ================================================================
  void AcController::initializeAsyncInference()
  {
    const size_t maxObservationSize = std::max(
        static_cast<size_t>(observationSize_ * stackSize_),
        static_cast<size_t>(observationSizeDance_ * stackSizeDance_));
    const size_t maxActionSize = std::max(static_cast<size_t>(actionsSize_),
                                          static_cast<size_t>(actionsSizeDance_));
    inferenceRuntime_.initializeBuffers(maxObservationSize, maxActionSize,
                                        0, 0);
  }

  void AcController::stopInferenceThread()
  {
    inferenceRuntime_.controllerActive.store(false, std::memory_order_release);
    inferenceRuntime_.inferenceRunning.store(false, std::memory_order_release);
    inferenceRuntime_.notifyInferenceStateChange();
    inferenceRuntime_.shutdownWatcherRunning.store(false, std::memory_order_release);
    inferenceRuntime_.shutdownWatcherCv.notify_all();
    requestInferenceTermination();
    if (inferenceRuntime_.shutdownWatcherThread.joinable()) {
      inferenceRuntime_.shutdownWatcherThread.join();
    }
    if (inferenceRuntime_.inferenceThread.joinable()) {
      inferenceRuntime_.inferenceThread.join();
    }
    // Flush a policy summary that may have been closed immediately before
    // shutdown, without relying on another timing-publisher iteration.
    publishCompletedPolicyInferenceStats();
    inferenceRuntime_.timingPublisherRunning.store(false, std::memory_order_release);
    if (inferenceRuntime_.timingPublisherThread.joinable()) {
      inferenceRuntime_.timingPublisherThread.join();
    }
  }

  void AcController::requestInferenceTermination()
  {
    bool expected = false;
    if (!inferenceRuntime_.terminationRequested.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
      return;
    }

    // ONNX Runtime explicitly permits this call from another thread to force
    // all Session::Run calls using this RunOptions instance to return.
    try {
      inferenceRunOptions_.SetTerminate();
    } catch (const Ort::Exception& exception) {
      inferenceRuntime_.terminationRequested.store(false, std::memory_order_release);
      ROS_ERROR_STREAM("[AcController] Failed to terminate in-flight inference during shutdown: "
                       << exception.what());
    }
  }

  void AcController::shutdownWatcherLoop()
  {
    pthread_setname_np(pthread_self(), "onnx_shutdown");
    std::unique_lock<std::mutex> lock(inferenceRuntime_.shutdownWatcherMutex);
    while (inferenceRuntime_.shutdownWatcherRunning.load(std::memory_order_acquire) && ros::ok()) {
      inferenceRuntime_.shutdownWatcherCv.wait_for(lock, std::chrono::milliseconds(10), [this]() {
        return !inferenceRuntime_.shutdownWatcherRunning.load(std::memory_order_acquire);
      });
    }
    const bool rosShuttingDown =
        inferenceRuntime_.shutdownWatcherRunning.load(std::memory_order_acquire) && !ros::ok();
    lock.unlock();
    if (rosShuttingDown) {
      requestInferenceTermination();
    }
  }

  bool AcController::warmupPolicies()
  {
    if (warmupRuns_ <= 0) return true;

    ActionPacket output;
    size_t actionCapacity = 0;
    for (const PolicyModeRegistration& mode : policyModes_) {
      actionCapacity = std::max(actionCapacity, mode.actions(*this).size());
    }
    output.actions.resize(actionCapacity, 0.0f);

    for (int i = 0; i < warmupRuns_; ++i) {
      for (const PolicyModeRegistration& mode : policyModes_) {
        if (!mode.warmup(*this, output)) {
          ROS_WARN("[AcController] %s warmup stopped after run %d", mode.name, i);
          return false;
        }
      }
    }
    ROS_INFO_STREAM("[AcController] Warmed up enabled policies with "
                    << warmupRuns_ << " runs each.");
    return true;
  }

  void AcController::inferenceLoop()
  {
    pthread_setname_np(pthread_self(), "ac_inference");
    bool realtimeSetupOk = true;
    if (inferenceCpuAffinity_ >= 0) {
      const long cpuCount = sysconf(_SC_NPROCESSORS_ONLN);
      if (cpuCount > 0 && inferenceCpuAffinity_ < cpuCount) {
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        CPU_SET(inferenceCpuAffinity_, &cpuSet);
        const int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet);
        if (ret != 0) {
          realtimeSetupOk = false;
          ROS_WARN("[AcController] Failed to pin inference thread to CPU %d: %s",
                   inferenceCpuAffinity_, std::strerror(ret));
        } else {
          ROS_INFO("[AcController] Inference thread pinned to CPU %d", inferenceCpuAffinity_);
        }
      } else {
        realtimeSetupOk = false;
        ROS_WARN("[AcController] Invalid inference_cpu_affinity=%d for %ld CPUs",
                 inferenceCpuAffinity_, cpuCount);
      }
    }

    // Priority 0 deliberately keeps the default SCHED_OTHER policy. Positive
    // values opt into a bounded priority below the control loop's FIFO 95.
    if (inferenceThreadPriority_ > 0) {
      sched_param inferenceSchedule{};
      inferenceSchedule.sched_priority = inferenceThreadPriority_;
      const int ret = pthread_setschedparam(
          pthread_self(), SCHED_FIFO, &inferenceSchedule);
      if (ret != 0) {
        realtimeSetupOk = false;
        ROS_WARN("[AcController] Failed to set inference thread SCHED_FIFO %d: %s. "
                 "Continuing with the existing scheduling policy.",
                 inferenceThreadPriority_, std::strerror(ret));
      } else {
        ROS_INFO("[AcController] Inference thread using SCHED_FIFO priority %d",
                 inferenceThreadPriority_);
      }
    }

    inferenceRuntime_.setupStatus.store(realtimeSetupOk ? 1 : -1, std::memory_order_release);

    uint64_t observedNotification =
        inferenceRuntime_.observationNotificationGeneration();
    while (inferenceRuntime_.inferenceRunning.load(std::memory_order_acquire)) {
      if (!inferenceRuntime_.controllerActive.load(std::memory_order_acquire)) {
        inferenceRuntime_.waitUntilInferenceActive();
        observedNotification =
            inferenceRuntime_.observationNotificationGeneration();
        continue;
      }

      const ObservationPacket* observation = inferenceRuntime_.consumeObservation();
      if (observation == nullptr) {
        inferenceRuntime_.waitForObservationNotification(observedNotification);
        observedNotification =
            inferenceRuntime_.observationNotificationGeneration();
        continue;
      }
      observedNotification =
          inferenceRuntime_.observationNotificationGeneration();
      if (!inferenceRuntime_.controllerActive.load(std::memory_order_acquire) ||
          observation->epoch != inferenceRuntime_.policyEpoch.load(std::memory_order_acquire)) {
        continue;
      }

      ActionPacket& result = inferenceRuntime_.beginAction();
      result.valid = false;
      result.policy = observation->policy;
      result.epoch = observation->epoch;
      result.sourceSequence = observation->sequence;
      result.observationTimestampNs = observation->timestampNs;
      result.estimatedVelocity = {{0.0f, 0.0f, 0.0f}};
      result.estimateValid = false;
      const bool ok = executePolicyTimed(*observation, result);

      inferenceRuntime_.publishAction();
      if (ok) {
        publishTrace(policyObservationPublisher_, observation->data, observation->size);
        if (observation->historySize > 0)
          publishTrace(policyHistoryPublisher_, observation->history, observation->historySize);
        if (observation->commandSize > 0)
          publishTrace(policyCommandPublisher_, observation->command, observation->commandSize);
        publishTrace(policyActionPublisher_, result.actions, result.size);
        const PolicyModeRegistration* mode = findPolicyMode(result.policy);
        if (mode != nullptr && mode->publishTrace != nullptr) mode->publishTrace(*this, result);
      }
    }
  }

  bool AcController::executePolicyTimed(const ObservationPacket& input, ActionPacket& output)
  {
    const int64_t startNs = steadyNowNs();
    const PolicyModeRegistration* mode = findPolicyMode(input.policy);
    const bool ok = mode != nullptr && mode->runner(*this, input, output);
    const int64_t completionNs = steadyNowNs();

    const int64_t inferenceUs = std::max<int64_t>(0, (completionNs - startNs) / 1000LL);
    output.inferenceTimeUs = static_cast<uint32_t>(inferenceUs);
    output.completionTimestampNs = completionNs;
    output.valid = ok;
    inferenceRuntime_.lastInferenceUs.store(output.inferenceTimeUs, std::memory_order_relaxed);
    if (!ok) inferenceRuntime_.inferenceFailures.fetch_add(1, std::memory_order_relaxed);
    if (tracksCumulativeInference(input.policy)) {
      accumulatePolicyInferenceStats(
          input.policy, input.epoch, output.inferenceTimeUs, ok);
    }

    if (enableTimingMetrics_) {
      InferenceTimingSample sample;
      sample.sequence = input.sequence;
      sample.policy = static_cast<double>(static_cast<uint8_t>(input.policy));
      sample.epoch = input.epoch;
      sample.observationNs = input.timestampNs;
      sample.startNs = startNs;
      sample.completionNs = completionNs;
      sample.inferenceUs = static_cast<double>(inferenceUs);
      sample.queueUs = static_cast<double>(startNs - input.timestampNs) / 1000.0;
      sample.endToEndUs = static_cast<double>(completionNs - input.timestampNs) / 1000.0;
      sample.success = ok ? 1.0 : 0.0;
      sample.asyncMode = asyncInferenceEnabled_ ? 1.0 : 0.0;
      if (!inferenceRuntime_.inferenceTimingBuffer.push(sample)) {
        inferenceRuntime_.inferenceTimingDropped.fetch_add(1, std::memory_order_relaxed);
      }
    }
    return ok;
  }

  void AcController::timingPublisherLoop()
  {
    pthread_setname_np(pthread_self(), "rl_timing_pub");
    constexpr size_t kMaxBatch = 256;
    constexpr size_t kInferenceColumns = 12;
    constexpr size_t kActionColumns = 10;

    while (inferenceRuntime_.timingPublisherRunning.load(std::memory_order_acquire)) {
      if (const TracePacket* trace = inferenceRuntime_.consumeSynchronousTrace()) {
        publishTrace(policyObservationPublisher_, trace->observation, trace->observationSize);
        if (trace->historySize > 0)
          publishTrace(policyHistoryPublisher_, trace->history, trace->historySize);
        if (trace->commandSize > 0)
          publishTrace(policyCommandPublisher_, trace->command, trace->commandSize);
        publishTrace(policyActionPublisher_, trace->action, trace->actionSize);
      }
      publishCompletedPolicyInferenceStats();
      if (!enableTimingMetrics_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      const bool publishInference = inferenceTimingPublisher_.getNumSubscribers() > 0;
      std_msgs::Float64MultiArray inferenceMessage;
      if (publishInference) {
        inferenceMessage.layout.dim.resize(2);
        inferenceMessage.layout.dim[0].label = "samples";
        inferenceMessage.layout.dim[1].label =
            "sequence,policy,epoch,observation_ns,start_ns,completion_ns,inference_us,queue_us,end_to_end_us,success,async_mode,dropped_total";
        inferenceMessage.layout.dim[1].size = kInferenceColumns;
        inferenceMessage.layout.dim[1].stride = kInferenceColumns;
        inferenceMessage.data.reserve(kInferenceColumns * 16);
      }
      const double inferenceDropped =
          static_cast<double>(inferenceRuntime_.inferenceTimingDropped.load(std::memory_order_relaxed));
      InferenceTimingSample inferenceSample;
      size_t inferenceRows = 0;
      while (inferenceRows < kMaxBatch && inferenceRuntime_.inferenceTimingBuffer.pop(inferenceSample)) {
        if (publishInference) {
          inferenceMessage.data.push_back(static_cast<double>(inferenceSample.sequence));
          inferenceMessage.data.push_back(inferenceSample.policy);
          inferenceMessage.data.push_back(static_cast<double>(inferenceSample.epoch));
          inferenceMessage.data.push_back(static_cast<double>(inferenceSample.observationNs));
          inferenceMessage.data.push_back(static_cast<double>(inferenceSample.startNs));
          inferenceMessage.data.push_back(static_cast<double>(inferenceSample.completionNs));
          inferenceMessage.data.push_back(inferenceSample.inferenceUs);
          inferenceMessage.data.push_back(inferenceSample.queueUs);
          inferenceMessage.data.push_back(inferenceSample.endToEndUs);
          inferenceMessage.data.push_back(inferenceSample.success);
          inferenceMessage.data.push_back(inferenceSample.asyncMode);
          inferenceMessage.data.push_back(inferenceDropped);
        }
        ++inferenceRows;
      }
      if (publishInference && inferenceRows > 0) {
        inferenceMessage.layout.dim[0].size = inferenceRows;
        inferenceMessage.layout.dim[0].stride = inferenceRows * kInferenceColumns;
        inferenceTimingPublisher_.publish(inferenceMessage);
      }

      const bool publishAction = actionTimingPublisher_.getNumSubscribers() > 0;
      std_msgs::Float64MultiArray actionMessage;
      if (publishAction) {
        actionMessage.layout.dim.resize(2);
        actionMessage.layout.dim[0].label = "samples";
        actionMessage.layout.dim[1].label =
            "sequence,policy,epoch,observation_ns,completion_ns,consumed_ns,completion_to_consume_us,observation_to_consume_us,async_mode,dropped_total";
        actionMessage.layout.dim[1].size = kActionColumns;
        actionMessage.layout.dim[1].stride = kActionColumns;
        actionMessage.data.reserve(kActionColumns * 16);
      }
      const double actionDropped =
          static_cast<double>(inferenceRuntime_.actionTimingDropped.load(std::memory_order_relaxed));
      ActionTimingSample actionSample;
      size_t actionRows = 0;
      while (actionRows < kMaxBatch && inferenceRuntime_.actionTimingBuffer.pop(actionSample)) {
        if (publishAction) {
          actionMessage.data.push_back(static_cast<double>(actionSample.sequence));
          actionMessage.data.push_back(actionSample.policy);
          actionMessage.data.push_back(static_cast<double>(actionSample.epoch));
          actionMessage.data.push_back(static_cast<double>(actionSample.observationNs));
          actionMessage.data.push_back(static_cast<double>(actionSample.completionNs));
          actionMessage.data.push_back(static_cast<double>(actionSample.consumedNs));
          actionMessage.data.push_back(actionSample.completionToConsumeUs);
          actionMessage.data.push_back(actionSample.observationToConsumeUs);
          actionMessage.data.push_back(actionSample.asyncMode);
          actionMessage.data.push_back(actionDropped);
        }
        ++actionRows;
      }
      if (publishAction && actionRows > 0) {
        actionMessage.layout.dim[0].size = actionRows;
        actionMessage.layout.dim[0].stride = actionRows * kActionColumns;
        actionTimingPublisher_.publish(actionMessage);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }



  bool AcController::runSynchronousPolicy(
      PolicyKind policy, const std::vector<tensor_element_t>& observation)
  {
    if (observation.size() > inferenceRuntime_.synchronousObservation.data.size()) return false;
    inferenceRuntime_.synchronousObservation.policy = policy;
    inferenceRuntime_.synchronousObservation.epoch = inferenceRuntime_.policyEpoch.load(std::memory_order_acquire);
    inferenceRuntime_.synchronousObservation.sequence = ++observationSequence_;
    inferenceRuntime_.synchronousObservation.timestampNs = steadyNowNs();
    inferenceRuntime_.synchronousObservation.size = observation.size();
    inferenceRuntime_.synchronousObservation.historySize = 0;
    inferenceRuntime_.synchronousObservation.commandSize = 0;
    std::copy(observation.begin(), observation.end(), inferenceRuntime_.synchronousObservation.data.begin());

    inferenceRuntime_.synchronousAction.valid = false;
    inferenceRuntime_.synchronousAction.policy = policy;
    inferenceRuntime_.synchronousAction.epoch = inferenceRuntime_.synchronousObservation.epoch;
    inferenceRuntime_.synchronousAction.sourceSequence = inferenceRuntime_.synchronousObservation.sequence;
    inferenceRuntime_.synchronousAction.observationTimestampNs = inferenceRuntime_.synchronousObservation.timestampNs;
    inferenceRuntime_.synchronousAction.estimatedVelocity = {{0.0f, 0.0f, 0.0f}};
    inferenceRuntime_.synchronousAction.estimateValid = false;
    if (!executePolicyTimed(inferenceRuntime_.synchronousObservation, inferenceRuntime_.synchronousAction)) return false;

    const bool applied = applyActionPacket(inferenceRuntime_.synchronousAction, policy);
    if (applied) {
      TracePacket& trace = inferenceRuntime_.beginSynchronousTrace();
      trace.observationSize = inferenceRuntime_.synchronousObservation.size;
      trace.actionSize = inferenceRuntime_.synchronousAction.size;
      trace.historySize = inferenceRuntime_.synchronousObservation.historySize;
      trace.commandSize = inferenceRuntime_.synchronousObservation.commandSize;
      std::copy_n(inferenceRuntime_.synchronousObservation.data.begin(), trace.observationSize, trace.observation.begin());
      std::copy_n(inferenceRuntime_.synchronousAction.actions.begin(), trace.actionSize, trace.action.begin());
      inferenceRuntime_.publishSynchronousTrace();
    }
    return applied;
  }

  void AcController::submitObservation(
      PolicyKind policy, const std::vector<tensor_element_t>& observation)
  {
    ObservationPacket& packet = inferenceRuntime_.beginObservation();
    if (observation.size() > packet.data.size()) {
      enqueueControlEvent(ControlEvent::OBSERVATION_BUFFER_OVERFLOW);
      return;
    }

    packet.policy = policy;
    packet.epoch = inferenceRuntime_.policyEpoch.load(std::memory_order_acquire);
    packet.sequence = ++observationSequence_;
    packet.timestampNs = steadyNowNs();
    packet.size = observation.size();
    packet.historySize = 0;
    packet.commandSize = 0;
    std::copy(observation.begin(), observation.end(), packet.data.begin());

    inferenceRuntime_.publishObservation();
  }

  bool AcController::consumeAction(PolicyKind expectedPolicy)
  {
    const ActionPacket* result = inferenceRuntime_.consumeAction();
    return result != nullptr && applyActionPacket(*result, expectedPolicy);
  }

  bool AcController::applyActionPacket(const ActionPacket& result, PolicyKind expectedPolicy)
  {
    if (!result.valid) return false;

    const uint64_t currentEpoch = inferenceRuntime_.policyEpoch.load(std::memory_order_acquire);
    if (result.epoch != currentEpoch || result.policy != expectedPolicy ||
        result.sourceSequence <= lastAppliedSequence_) {
      return false;
    }

    const int64_t consumedNs = steadyNowNs();
    const int64_t ageNs = consumedNs - result.observationTimestampNs;
    if (actionTimeoutMs_ > 0 && ageNs > static_cast<int64_t>(actionTimeoutMs_) * 1000000LL) {
      return false;
    }

    const PolicyModeRegistration* mode = findPolicyMode(expectedPolicy);
    if (mode == nullptr) return false;
    std::vector<tensor_element_t>* destination = &mode->actions(*this);
    if (result.size != destination->size() || result.size > result.actions.size()) {
      enqueueControlEvent(ControlEvent::ACTION_SIZE_MISMATCH);
      return false;
    }

    std::copy_n(result.actions.begin(), result.size, destination->begin());
    lastAppliedSequence_ = result.sourceSequence;
    lastAppliedObservationTimestampNs_ = result.observationTimestampNs;
    if (enableTimingMetrics_) {
      ActionTimingSample sample;
      sample.sequence = result.sourceSequence;
      sample.policy = static_cast<double>(static_cast<uint8_t>(result.policy));
      sample.epoch = result.epoch;
      sample.observationNs = result.observationTimestampNs;
      sample.completionNs = result.completionTimestampNs;
      sample.consumedNs = consumedNs;
      sample.completionToConsumeUs =
          static_cast<double>(consumedNs - result.completionTimestampNs) / 1000.0;
      sample.observationToConsumeUs = static_cast<double>(ageNs) / 1000.0;
      sample.asyncMode = asyncInferenceEnabled_ ? 1.0 : 0.0;
      if (!inferenceRuntime_.actionTimingBuffer.push(sample)) {
        inferenceRuntime_.actionTimingDropped.fetch_add(1, std::memory_order_relaxed);
      }
    }
    return true;
  }

  void AcController::enterPolicyMode(PolicyKind policy)
  {
    if (activePolicy_ == policy) return;

    const int64_t entryNs = steadyNowNs();
    if (tracksCumulativeInference(activePolicy_)) {
      endPolicyInferenceStats(activePolicy_,
                              inferenceRuntime_.policyEpoch.load(std::memory_order_acquire), entryNs);
    }

    activePolicy_ = policy;
    const uint64_t epoch = inferenceRuntime_.policyEpoch.fetch_add(1, std::memory_order_acq_rel) + 1;
    lastAppliedSequence_ = 0;
    policyEntryTimestampNs_ = entryNs;
    lastAppliedObservationTimestampNs_ = 0;
    isfirstRecObs_ = true;

    const PolicyModeRegistration* mode = findPolicyMode(policy);
    if (mode != nullptr) {
      std::vector<tensor_element_t>& actions = mode->actions(*this);
      std::fill(actions.begin(), actions.end(), 0.0f);
      mode->lastActions(*this).setZero();
      mode->onEnter(*this);
    }
    if (tracksCumulativeInference(policy)) {
      beginPolicyInferenceStats(policy, epoch, entryNs);
    }
  }

  void AcController::leavePolicyMode()
  {
    if (activePolicy_ == PolicyKind::NONE) return;
    if (tracksCumulativeInference(activePolicy_)) {
      endPolicyInferenceStats(activePolicy_,
                              inferenceRuntime_.policyEpoch.load(std::memory_order_acquire), steadyNowNs());
    }
    activePolicy_ = PolicyKind::NONE;
    inferenceRuntime_.policyEpoch.fetch_add(1, std::memory_order_acq_rel);
    lastAppliedSequence_ = 0;
    policyEntryTimestampNs_ = 0;
    lastAppliedObservationTimestampNs_ = 0;
  }

  bool AcController::tracksCumulativeInference(PolicyKind policy) const noexcept
  {
    const PolicyModeRegistration* mode = findPolicyMode(policy);
    return mode != nullptr && mode->cumulativeInferenceStats;
  }

  const char* AcController::policyKindName(PolicyKind policy) const noexcept
  {
    if (policy == PolicyKind::NONE) return "NONE";
    const PolicyModeRegistration* mode = findPolicyMode(policy);
    return mode == nullptr ? "UNKNOWN" : mode->name;
  }

  void AcController::beginPolicyInferenceStats(
      PolicyKind policy, uint64_t epoch, int64_t startNs) noexcept
  {
    PolicyInferenceStats& stats = policyInferenceStats_[epoch % kPolicyInferenceStatsSlots];
    stats.ended.store(false, std::memory_order_relaxed);
    stats.totalInferenceUs.store(0, std::memory_order_relaxed);
    stats.sampleCount.store(0, std::memory_order_relaxed);
    stats.failureCount.store(0, std::memory_order_relaxed);
    stats.activeWriters.store(0, std::memory_order_relaxed);
    stats.startNs.store(startNs, std::memory_order_relaxed);
    stats.endNs.store(0, std::memory_order_relaxed);
    stats.policy.store(static_cast<uint8_t>(policy), std::memory_order_relaxed);
    stats.epoch.store(epoch, std::memory_order_release);
  }

  void AcController::endPolicyInferenceStats(
      PolicyKind policy, uint64_t epoch, int64_t endNs) noexcept
  {
    PolicyInferenceStats& stats = policyInferenceStats_[epoch % kPolicyInferenceStatsSlots];
    if (stats.epoch.load(std::memory_order_acquire) != epoch ||
        stats.policy.load(std::memory_order_acquire) != static_cast<uint8_t>(policy)) return;
    stats.endNs.store(endNs, std::memory_order_relaxed);
    stats.ended.store(true, std::memory_order_release);
  }

  void AcController::accumulatePolicyInferenceStats(
      PolicyKind policy, uint64_t epoch, uint64_t inferenceUs, bool success) noexcept
  {
    PolicyInferenceStats& stats = policyInferenceStats_[epoch % kPolicyInferenceStatsSlots];
    stats.activeWriters.fetch_add(1, std::memory_order_acq_rel);
    if (stats.epoch.load(std::memory_order_acquire) == epoch &&
        stats.policy.load(std::memory_order_acquire) == static_cast<uint8_t>(policy) &&
        !stats.ended.load(std::memory_order_acquire)) {
      stats.totalInferenceUs.fetch_add(inferenceUs, std::memory_order_relaxed);
      stats.sampleCount.fetch_add(1, std::memory_order_relaxed);
      if (!success) stats.failureCount.fetch_add(1, std::memory_order_relaxed);
    }
    stats.activeWriters.fetch_sub(1, std::memory_order_release);
  }

  void AcController::publishCompletedPolicyInferenceStats()
  {
    for (PolicyInferenceStats& stats : policyInferenceStats_) {
      if (!stats.ended.load(std::memory_order_acquire) ||
          stats.activeWriters.load(std::memory_order_acquire) != 0) {
        continue;
      }

      bool expectedEnded = true;
      if (!stats.ended.compare_exchange_strong(
              expectedEnded, false, std::memory_order_acq_rel)) {
        continue;
      }

      const uint64_t epoch = stats.epoch.load(std::memory_order_acquire);
      if (epoch == 0) continue;
      const PolicyKind policy = static_cast<PolicyKind>(
          stats.policy.load(std::memory_order_relaxed));
      const uint64_t totalUs = stats.totalInferenceUs.load(std::memory_order_relaxed);
      const uint64_t count = stats.sampleCount.load(std::memory_order_relaxed);
      const uint64_t failures = stats.failureCount.load(std::memory_order_relaxed);
      const int64_t startNs = stats.startNs.load(std::memory_order_relaxed);
      const int64_t endNs = stats.endNs.load(std::memory_order_relaxed);
      const double durationSec = endNs > startNs
          ? static_cast<double>(endNs - startNs) / 1.0e9 : 0.0;
      const double meanUs = count > 0
          ? static_cast<double>(totalUs) / static_cast<double>(count) : 0.0;

      ROS_INFO("[AcController] %s inference summary: epoch=%llu, samples=%llu, "
               "cumulative=%.3f ms, mean=%.3f us, failures=%llu, duration=%.3f s, mode=%s",
               policyKindName(policy),
               static_cast<unsigned long long>(epoch),
               static_cast<unsigned long long>(count),
               static_cast<double>(totalUs) / 1000.0,
               meanUs,
               static_cast<unsigned long long>(failures),
               durationSec,
               asyncInferenceEnabled_ ? "async" : "sync");
      stats.epoch.store(0, std::memory_order_release);
    }
  }

  bool AcController::actionTimedOut() const
  {
    if (actionTimeoutMs_ <= 0 || activePolicy_ == PolicyKind::NONE) return false;
    const int64_t referenceTimestamp = lastAppliedObservationTimestampNs_ != 0
        ? lastAppliedObservationTimestampNs_
        : policyEntryTimestampNs_;
    return referenceTimestamp != 0 &&
        steadyNowNs() - referenceTimestamp > static_cast<int64_t>(actionTimeoutMs_) * 1000000LL;
  }

  // ================================================================
  //  pdtest / CSV trajectory
  // ================================================================
  void AcController::buildControlJointMaps()
  {
    const auto& policy_joint_names = walkPolicy_.metadata().jointNames();
    const auto& default_joint_pos = walkPolicy_.metadata().defaultPosition();
    const auto& joint_stiffness = walkPolicy_.metadata().stiffness();
    const auto& joint_damping = walkPolicy_.metadata().damping();

    controlJointMaps_.default_joint_angles.assign(jointNames_.size(), 0.0);
    controlJointMaps_.joint_stiffness.assign(jointNames_.size(), 0.0);
    controlJointMaps_.joint_damping.assign(jointNames_.size(), 0.0);
    for (size_t i = 0; i < jointNames_.size(); ++i)
    {
      const auto it = std::find(policy_joint_names.begin(), policy_joint_names.end(),
                                jointNames_[i]);
      if (it == policy_joint_names.end())
      {
        ROS_WARN_STREAM("[AcController] control joint not found in walk metadata: " << jointNames_[i]);
        continue;
      }
      const size_t policy_idx = static_cast<size_t>(std::distance(policy_joint_names.begin(), it));
      if (policy_idx < default_joint_pos.size())
      {
        controlJointMaps_.default_joint_angles[i] = default_joint_pos[policy_idx];
      }
      if (policy_idx < joint_stiffness.size())
      {
        controlJointMaps_.joint_stiffness[i] = joint_stiffness[policy_idx];
      }
      if (policy_idx < joint_damping.size())
      {
        controlJointMaps_.joint_damping[i] = joint_damping[policy_idx];
      }
    }
  }

  void AcController::applyPdTestDefaultJointAngles(ros::NodeHandle& nh)
  {
    bool from_policy = true;
    nh.param("/LeggedRobotCfg/pdtest/default_joint_angles_from_policy", from_policy, true);
    if (from_policy)
    {
      return;
    }
    std::fill(controlJointMaps_.default_joint_angles.begin(),
              controlJointMaps_.default_joint_angles.end(), 0.0);
    ROS_INFO("[AcController] pdtest default_joint_angles: all 0 (from_policy=false)");
  }

  void AcController::applyPdTestAnklePdOverride(ros::NodeHandle& nh)
  {
    bool enabled = false;
    nh.param("/LeggedRobotCfg/pdtest/ankle_pd_override/enabled", enabled, false);
    if (!enabled)
    {
      return;
    }

    double kp = -1.0;
    double kd = -1.0;
    nh.param("/LeggedRobotCfg/pdtest/ankle_pd_override/kp", kp, -1.0);
    nh.param("/LeggedRobotCfg/pdtest/ankle_pd_override/kd", kd, -1.0);
    if (kp < 0.0 || kd < 0.0)
    {
      ROS_WARN("[AcController] pdtest ankle_pd_override enabled but kp/kd invalid; skipping");
      return;
    }

    static const char* kAnkleJoints[] = {
        "l_leg_ankle_pitch_joint", "l_leg_ankle_roll_joint",
        "r_leg_ankle_pitch_joint", "r_leg_ankle_roll_joint",
    };
    for (const char* joint_name : kAnkleJoints)
    {
      const auto it = std::find(jointNames_.begin(), jointNames_.end(), joint_name);
      if (it == jointNames_.end())
      {
        ROS_WARN_STREAM("[AcController] pdtest ankle_pd_override: joint not found: " << joint_name);
        continue;
      }
      const size_t idx = static_cast<size_t>(std::distance(jointNames_.begin(), it));
      controlJointMaps_.joint_stiffness[idx] = kp;
      controlJointMaps_.joint_damping[idx] = kd;
      ROS_INFO_STREAM("[AcController] pdtest ankle_pd_override: " << joint_name
                      << " kp=" << kp << " kd=" << kd);
    }
  }

  bool AcController::loadPdTestConfig(ros::NodeHandle& nh)
  {
    applyPdTestDefaultJointAngles(nh);
    applyPdTestAnklePdOverride(nh);
    csvDiagnosticLogger_.configure(nh, jointNames_);
    csv_log_start_pending_ = csvDiagnosticLogger_.loggingEnabled();
    return csvTrajectoryPlayer_.configure(nh, jointNames_, controlJointMaps_);
  }

  void AcController::endCsvPlaybackSession()
  {
    csvPlaybackSessionActive_ = false;
  }

  void AcController::playCsvTrajectory()
  {
    constexpr double kHwDt = 0.002;
    const bool is_sample_step = true;
    std::vector<double> current_pos(hybridJointHandles_.size());
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
      current_pos[i] = hybridJointHandles_[i].getPosition();
    }
    csvTrajectoryPlayer_.update(kHwDt, is_sample_step, current_pos);

    if (csvTrajectoryPlayer_.playbackStarted())
    {
      if (csv_log_start_pending_)
      {
        csvDiagnosticLogger_.maybeStart(csvTrajectoryPlayer_.resolvedCsvPath(),
                                        csvTrajectoryPlayer_.sampleInterval());
        csv_log_start_pending_ = false;
      }
    }

    for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
      hybridJointHandles_[i].setCommand(
          csvTrajectoryPlayer_.posDes(i),
          csvTrajectoryPlayer_.velDes(i),
          csvTrajectoryPlayer_.stiffness(i),
          csvTrajectoryPlayer_.damping(i),
          0.0);
    }

    if (csvTrajectoryPlayer_.playbackStarted())
    {
      csvDiagnosticLogger_.writeSampleIfActive(hybridJointHandles_, csvTrajectoryPlayer_, kHwDt);
    }

    if (csvTrajectoryPlayer_.finished())
    {
      csvDiagnosticLogger_.finalize("playback finished");
    }
  }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::AcControllerPlugin, controller_interface::ControllerBase)
