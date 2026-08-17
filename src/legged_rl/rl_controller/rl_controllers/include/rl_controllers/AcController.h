#pragma once
#include "rl_controllers/RLControllerBase.h"
#include "rl_controllers/LatestTripleBuffer.h"
#include "rl_controllers/PolicyRuntimeConfig.h"
#include "rl_controllers/PolicyRuntimeTypes.h"
#include "rl_controllers/PolicyInferenceRuntimeState.h"
#include "rl_controllers/policies/WalkPolicy.h"
#include "rl_controllers/policies/DancePolicy.h"
#include "rl_controllers/PdTrajectoryPlayer.h"
#include "rl_controllers/PdTrajectoryDiagnosticLogger.h"
#include <legged_common/SpscRingBuffer.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace legged
{
  template<PolicyKind Kind>
  struct PolicyModeRegistrationBuilder;

  class AcController : public RLControllerBase
  {
    using tensor_element_t = float;
  public:
    AcController();
    ~AcController() override;

    bool init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH) override;
    void starting(const ros::Time &time) override;
    void stopping(const ros::Time &time) override;

  protected:
    bool loadModel(ros::NodeHandle &nh) override;
    bool loadRLCfg(ros::NodeHandle &nh) override;
    bool loadMotions(ros::NodeHandle &nh) override;
    void computeObservation() override;
    bool controlRequestAllowed(ControlRequestId requestId) const noexcept override;
    // AcController provides reusable mode implementations while remaining an
    // abstract policy-controller contract. The plugin's concrete adapter is
    // intentionally private to the implementation.
    void handleWalkMode() override = 0;
    void handleDanceMode() override = 0;
    void handleDownMode() override = 0;
    void handleUpMode() override = 0;
    void handleDefaultMode() override = 0;
    void handleLieMode() override = 0;
    void handleStandMode() override = 0;

  private:
    using PolicyRunner = bool (*)(AcController&, const ObservationPacket&, ActionPacket&);
    using ActionBuffer = std::vector<tensor_element_t>& (*)(AcController&);
    using LastActionBuffer = vector_t& (*)(AcController&);
    using PolicyEntryHook = void (*)(AcController&);
    using PolicyResetHook = void (*)(AcController&);
    using PolicyWarmup = bool (*)(AcController&, ActionPacket&);
    using PolicyTraceHook = void (*)(AcController&, const ActionPacket&);
    using PolicyMotionLoader = bool (*)(AcController&, ros::NodeHandle&);

    struct PolicyModeRegistration
    {
      PolicyKind policy;
      const char* name;
      PolicyRunner runner;
      ActionBuffer actions;
      LastActionBuffer lastActions;
      PolicyEntryHook onEnter;
      PolicyResetHook onReset;
      PolicyMotionLoader loadMotion;
      PolicyWarmup warmup;
      PolicyTraceHook publishTrace;
      bool cumulativeInferenceStats;
    };

    static constexpr size_t kPolicyModeCount = 4;
    static_assert(static_cast<size_t>(PolicyKind::UP) == kPolicyModeCount,
                  "PolicyKind values must stay contiguous for registry lookup");
    static bool registerPolicyMode(const PolicyModeRegistration& mode) noexcept;
    static std::array<PolicyModeRegistration, kPolicyModeCount>& policyModeCatalog() noexcept;
    const PolicyModeRegistration* findPolicyMode(PolicyKind policy) const noexcept;
    template<PolicyKind Kind>
    friend struct PolicyModeRegistrationBuilder;

    void initializePolicyRosInterfaces();
    bool loadPolicyRuntimeConfig(ros::NodeHandle& nh);
    void initializePolicyState();
    struct PolicyInferenceStats
    {
      std::atomic<uint64_t> epoch{0};
      std::atomic<uint8_t> policy{static_cast<uint8_t>(PolicyKind::NONE)};
      std::atomic<uint64_t> totalInferenceUs{0};
      std::atomic<uint64_t> sampleCount{0};
      std::atomic<uint64_t> failureCount{0};
      std::atomic<uint32_t> activeWriters{0};
      std::atomic<int64_t> startNs{0};
      std::atomic<int64_t> endNs{0};
      std::atomic_bool ended{false};
    };

    void transformBaseOriToTorsoOri(const Eigen::Quaterniond& base_quat,
                                    const std::array<double, 3>& waist_joint_angles,
                                    Eigen::Quaterniond& torso_quat);
    void initializeAsyncInference();
    void stopInferenceThread();
    void shutdownWatcherLoop();
    void requestInferenceTermination();
    void inferenceLoop();
    void timingPublisherLoop();
    bool warmupPolicies();
    bool executePolicyTimed(const ObservationPacket& input, ActionPacket& output);
    bool runWalkPolicy(const ObservationPacket& input, ActionPacket& output);
    bool runDancePolicy(const ObservationPacket& input, ActionPacket& output);
    bool runDownPolicy(const ObservationPacket& input, ActionPacket& output);
    bool runUpPolicy(const ObservationPacket& input, ActionPacket& output);
    bool runSynchronousPolicy(PolicyKind policy, const std::vector<tensor_element_t>& observation);
    void submitObservation(PolicyKind policy, const std::vector<tensor_element_t>& observation);
    bool consumeAction(PolicyKind expectedPolicy);
    bool applyActionPacket(const ActionPacket& result, PolicyKind expectedPolicy);
    void enterPolicyMode(PolicyKind policy);
    void leavePolicyMode();
    bool tracksCumulativeInference(PolicyKind policy) const noexcept;
    const char* policyKindName(PolicyKind policy) const noexcept;
    void beginPolicyInferenceStats(PolicyKind policy, uint64_t epoch,
                                   int64_t startNs) noexcept;
    void endPolicyInferenceStats(PolicyKind policy, uint64_t epoch,
                                 int64_t endNs) noexcept;
    void accumulatePolicyInferenceStats(PolicyKind policy, uint64_t epoch,
                                        uint64_t inferenceUs, bool success) noexcept;
    void publishCompletedPolicyInferenceStats();
    bool actionTimedOut() const;
    static int64_t steadyNowNs();
    bool computeObservationDance(double motionTimeSec);
    bool computeWalkObservation();
    void resetWalkPlaybackClock() noexcept;
    void resetDancePlaybackClock() noexcept;
    void resetDownPlaybackClock() noexcept;
    void resetUpPlaybackClock() noexcept;
    void beginUpPreparation() noexcept;
    bool handleUpPreparation() noexcept;

    struct MotionPolicyState
    {
      DancePolicy policy;
      vector_t lastActions;
      vector_t defaultJointAngles;
      std::vector<tensor_element_t> actions;
      std::vector<tensor_element_t> observations;
      Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> historyBuffer;
      vector_t proprioObs;
      int timeStep{0};
      bool dampingAfterCompletion{false};
    };

    void handleMotionMode(PolicyKind kind, MotionPolicyState& state,
                          ControlEvent completeEvent, ControlEvent policyErrorEvent,
                          ControlEvent timeoutEvent, ControlEvent proprioErrorEvent);
    bool runMotionPolicy(MotionPolicyState& state, ControlEvent policyErrorEvent,
                         const ObservationPacket& input, ActionPacket& output);
    void resetMotionPlaybackClock(MotionPolicyState& state) noexcept;
    bool computeObservationMotion(MotionPolicyState& state, double motionTimeSec,
                                  ControlEvent proprioErrorEvent, const char* modeName);
    bool loadMotionFile(MotionPolicyState& state, ros::NodeHandle& nh,
                        const char* parameterName, const char* modeName);
    void beginDownCompletionDamping() noexcept;
    void handleDownCompletionDamping() noexcept;

  private:
    // ===================== walk/dance policy =====================
    std::shared_ptr<Ort::Env> onnxEnv_;
    WalkPolicy walkPolicy_;
    MotionPolicyState dance_;
    MotionPolicyState down_;
    MotionPolicyState up_;

    vector_t lastActions_;
    vector_t defaultJointAngles_;

    int actionsSize_;
    int actionsSizeDance_;
    int observationSize_;
    int observationSizeDance_;
    int stackSize_;
    int stackSizeDance_;
    bool walkUsesProjectedGravity_{false};
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> policyObservations_;
    // Shared with the shutdown path so an in-flight Session::Run can be
    // interrupted before joining the inference thread.
    Ort::RunOptions inferenceRunOptions_;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
    vector_t walkProprioObs_;

    ros::Publisher policyObservationPublisher_;
    ros::Publisher policyActionPublisher_;
    ros::Publisher velEstimatePublisher_;
    ros::Publisher policyHistoryPublisher_;
    ros::Publisher policyCommandPublisher_;
    ros::Publisher inferenceTimingPublisher_;
    ros::Publisher actionTimingPublisher_;

    PolicyInferenceRuntimeState inferenceRuntime_;
    std::array<PolicyModeRegistration, kPolicyModeCount> policyModes_;
    static constexpr size_t kPolicyInferenceStatsSlots = 4;
    std::array<PolicyInferenceStats, kPolicyInferenceStatsSlots> policyInferenceStats_{};
    PolicyKind activePolicy_{PolicyKind::NONE};
    uint64_t observationSequence_{0};
    uint64_t lastAppliedSequence_{0};
    int64_t policyEntryTimestampNs_{0};
    int64_t lastAppliedObservationTimestampNs_{0};
    int actionTimeoutMs_{100};
    int inferenceCpuAffinity_{-1};
    int inferenceThreadPriority_{20};
    int warmupRuns_{20};
    int64_t upTransitionDurationNs_{2000000000LL};
    int64_t upPolicyBlendDurationNs_{500000000LL};
    int64_t downTorqueFadeDurationNs_{1000000000LL};
    bool asyncInferenceEnabled_{true};
    bool enableTimingMetrics_{false};
    bool requireRealtime_{false};
    static constexpr scalar_t kDownCompletionDamping = 0.1;

    vector_t upTransitionStartAngles_;
    vector_t upPolicyBlendStartActions_;
    int64_t upTransitionStartNs_{0};
    int64_t upPolicyBlendStartNs_{0};
    bool upPreparationActive_{false};
    bool upGainsApplied_{false};
    bool upPolicyBlendActive_{false};

    static constexpr size_t kMaxTransitionDof = 24;
    std::array<double, kMaxTransitionDof> downFadePositionDesired_{};
    std::array<double, kMaxTransitionDof> downFadeVelocityDesired_{};
    std::array<double, kMaxTransitionDof> downFadeKp_{};
    std::array<double, kMaxTransitionDof> downFadeKd_{};
    std::array<double, kMaxTransitionDof> downFadeFeedforward_{};
    int64_t downTorqueFadeStartNs_{0};
    bool downTorqueFadeActive_{false};

    void publishTrace(const ros::Publisher& publisher,
                      const std::vector<tensor_element_t>& data,
                      size_t size) const;

    // ===================== pdtest / CSV trajectory =====================
    bool loadPdTestConfig(ros::NodeHandle& nh);
    void buildControlJointMaps();
    void applyPdTestDefaultJointAngles(ros::NodeHandle& nh);
    void applyPdTestAnklePdOverride(ros::NodeHandle& nh);
    void playCsvTrajectory();
    void endCsvPlaybackSession();

    PdTrajectoryPlayer csvTrajectoryPlayer_;
    PdTrajectoryDiagnosticLogger csvDiagnosticLogger_;
    PdTrajectoryControlMaps controlJointMaps_;
    bool csv_log_start_pending_{false};
    bool csvPlaybackSessionActive_{false};
  };

} // namespace legged
