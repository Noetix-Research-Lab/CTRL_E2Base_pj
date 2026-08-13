#include "rl_controllers/RLControllerBase.h"
#include <string.h>
#include "rl_controllers/RotationTools.h"
#include "rl_controllers/utilities.h"
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <pthread.h>

namespace legged
{
  RLControllerBase::~RLControllerBase()
  {
    stopAnalysisPublisher();
  }

  namespace
  {
    int64_t steadyNowNs()
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
    }

  } // namespace

  bool RLControllerBase::init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH)
  {
    for (auto& receivedNs : requestLastReceivedNs_) {
      receivedNs.store(0, std::memory_order_relaxed);
    }
    std::vector<std::string> defaultControlJointNames{
        "l_leg_hip_pitch_joint", "l_leg_hip_roll_joint", "l_leg_hip_yaw_joint",
        "l_leg_knee_joint", "l_leg_ankle_pitch_joint", "l_leg_ankle_roll_joint",
        "r_leg_hip_pitch_joint", "r_leg_hip_roll_joint", "r_leg_hip_yaw_joint",
        "r_leg_knee_joint", "r_leg_ankle_pitch_joint", "r_leg_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "l_arm_shoulder_pitch_joint", "l_arm_shoulder_roll_joint",
        "l_arm_shoulder_yaw_joint", "l_arm_elbow_joint",
        "r_arm_shoulder_pitch_joint", "r_arm_shoulder_roll_joint",
        "r_arm_shoulder_yaw_joint", "r_arm_elbow_joint"
      };
    std::vector<std::string> defaultPolicyJointNames = defaultControlJointNames;
    ros::NodeHandle nh;
    nh.param<int>("/LeggedRobotCfg/robot/actuated_dof", actuatedDofNum_, 23);

    jointNames_ = defaultControlJointNames;
    policyJointNames_ = defaultPolicyJointNames;
    nh.getParam("/LeggedRobotCfg/robot/control_joint_names", jointNames_);
    nh.getParam("/LeggedRobotCfg/robot/policy_joint_names", policyJointNames_);
    actuatedDofNum_ = static_cast<int>(jointNames_.size());
    policyDofNum_ = static_cast<int>(policyJointNames_.size());

    if (actuatedDofNum_ != policyDofNum_) {
      ROS_ERROR_STREAM("[RLControllerBase] control_joint_names size (" << actuatedDofNum_
                       << ") must match policy_joint_names size (" << policyDofNum_ << ")");
      return false;
    }
    ROS_INFO_STREAM("[RLControllerBase] robot dof=" << actuatedDofNum_
                    << " policy dof=" << policyDofNum_);
    if (actuatedDofNum_ > static_cast<int>(kMaxAnalysisDof)) {
      ROS_ERROR_STREAM("[RLControllerBase] Too many joints for analysis buffer: "
                       << actuatedDofNum_ << " > " << kMaxAnalysisDof);
      return false;
    }

    if (!loadModel(controllerNH)) {
      ROS_ERROR_STREAM("[RLControllerBase] Failed to load the model.");
      return false;
    }
    if (!loadRLCfg(controllerNH)) {
      ROS_ERROR_STREAM("[RLControllerBase] Failed to load the rl config.");
      return false;
    }
    if (!loadMotions(controllerNH)) {
      ROS_ERROR_STREAM("[RLControllerBase] Failed to load the motions.");
      return false;
    }

    standJointAngles_.resize(actuatedDofNum_);
    lieJointAngles_.resize(actuatedDofNum_);
    auto &StandState = standjointState_;
    auto &LieState = liejointState_;

    joint_dim_ = actuatedDofNum_;
    nh.getParam("/Kd_config/kd", cfg_kd);

    urdf::Model urdfModel;
    std::string urdfParamName;
    if (nh.hasParam("legged_robot_description")) {
      urdfParamName = "legged_robot_description";
    } else if (nh.hasParam("robot_description")) {
      urdfParamName = "robot_description";
    }
    if (urdfParamName.empty() || !urdfModel.initParam(urdfParamName)) {
      ROS_WARN_STREAM("[LeggedRobotVisualizer] Could not read URDF from "
                      << "\"legged_robot_description\" or \"robot_description\"");
    } else {
      KDL::Tree kdlTree;
      kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);
      robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    }

    realJointPosPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/real_joint_pos", 1);
    realJointVelPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/real_joint_vel", 1);
    realTorquePublisher_   = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/real_torque", 1);
    realImuAngularVelPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/imu_angular_vel", 1);
    realImuLinearAccPublisher_  = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/imu_linear_acc", 1);
    realImuEulerXyzPulbisher    = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/imu_euler_xyz", 1);
    outputPlannedJointPosPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_planned_joint_pos", 1);
    outputPlannedJointVelPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_planned_joint_vel", 1);
    outputPlannedTorquePublisher_   = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_planned_torque", 1);
    outputCommandKpPublisher_       = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_command_kp", 1);
    outputCommandKdPublisher_       = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_command_kd", 1);
    outputCommandFfPublisher_       = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_command_ff", 1);
    modeCommandPublisher_           = nh.advertise<std_msgs::Float64MultiArray>("data_analysis/rl_mode_command", 1);
    nh.param<bool>("/enable_state_publish", enableStatePublish_, true);
    nh.param<bool>("/enable_tf_publish", enableTfPublish_, true);
    nh.param<int>("/data_publish_decimation", dataPublishDecimation_, 10);
    nh.param<int>("/tf_publish_decimation", tfPublishDecimation_, 10);
    controllerNH.param<bool>("enable_state_publish", enableStatePublish_, enableStatePublish_);
    controllerNH.param<bool>("enable_tf_publish", enableTfPublish_, enableTfPublish_);
    controllerNH.param<int>("data_publish_decimation", dataPublishDecimation_, dataPublishDecimation_);
    controllerNH.param<int>("tf_publish_decimation", tfPublishDecimation_, tfPublishDecimation_);
    dataPublishDecimation_ = std::max(1, dataPublishDecimation_);
    tfPublishDecimation_ = std::max(1, tfPublishDecimation_);
    double cmdVelTimeoutSec = 0.2;
    nh.param<double>("/LeggedRobotCfg/control/cmd_vel_timeout", cmdVelTimeoutSec, cmdVelTimeoutSec);
    controllerNH.param<double>("cmd_vel_timeout", cmdVelTimeoutSec, cmdVelTimeoutSec);
    if (!std::isfinite(cmdVelTimeoutSec) || cmdVelTimeoutSec <= 0.0 || cmdVelTimeoutSec > 60.0) {
      ROS_WARN("[RLControllerBase] Invalid cmd_vel_timeout=%.6f; using 0.2 s", cmdVelTimeoutSec);
      cmdVelTimeoutSec = 0.2;
    }
    cmdVelTimeoutNs_ = static_cast<int64_t>(cmdVelTimeoutSec * 1e9);

    for (int i = 0; i < actuatedDofNum_; ++i) {
      auto standIt = StandState.find(jointNames_[i]);
      auto lieIt = LieState.find(jointNames_[i]);
      if (standIt == StandState.end() || lieIt == LieState.end()) {
        ROS_ERROR_STREAM("[RLControllerBase] Unknown joint in control_joint_names: " << jointNames_[i]);
        return false;
      }
      standJointAngles_(i) = standIt->second;
      lieJointAngles_(i) = lieIt->second;
    }

    auto *hybridJointInterface = robotHw->get<HybridJointInterface>();
    for (const auto &jointName : jointNames_)
      hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName));
    for (const auto &jointName : policyJointNames_)
      policyJointHandles_.push_back(hybridJointInterface->getHandle(jointName));
    joint_positions_msg_.clear();
    for (const auto &jointName : jointNames_) {
      joint_positions_msg_.emplace(jointName, 0.0);
    }

    imuSensorHandles_ = robotHw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

    propri_.jointPos.resize(actuatedDofNum_);
    propri_.jointVel.resize(actuatedDofNum_);
    policyPropri_.jointPos.resize(policyDofNum_);
    policyPropri_.jointVel.resize(policyDofNum_);
    propri_.jointPos.setZero();
    propri_.jointVel.setZero();
    policyPropri_.jointPos.setZero();
    policyPropri_.jointVel.setZero();
    currentJointAngles_.resize(hybridJointHandles_.size());

    resizeAnalysisMessages();

    server_ptr_ = std::make_unique<dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>>(
                      ros::NodeHandle("controller"));
    dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>::CallbackType dynamicCallback;
    dynamicCallback = boost::bind(&RLControllerBase::dynamicParamCallback, this, _1, _2);
    server_ptr_->setCallback(dynamicCallback);

    cmdVelSub_  = controllerNH.subscribe("/cmd_vel", 1, &RLControllerBase::cmdVelCallback, this);
    joyInfoSub_ = controllerNH.subscribe("/joy", 1000, &RLControllerBase::joyInfoCallback, this);
    joystickConnectedSub_ = controllerNH.subscribe(
        "/joystick_connected", 1, &RLControllerBase::joystickConnectedCallback, this);
    switchCtrlClient_ = controllerNH.serviceClient<controller_manager_msgs::SwitchController>(
                            "/controller_manager/switch_controller");

    auto emergencyStopCallback = [this](const std_msgs::Float32::ConstPtr &msg){
      (void)msg;
      latchEmergencyStop();
    };
    emgStopSub_ = controllerNH.subscribe<std_msgs::Float32>("/emergency_stop", 1, emergencyStopCallback);

    // Reset is intentionally a separate command. Releasing the emergency-stop
    // source never clears the latch and reset does not restart control.
    auto emergencyStopResetCallback = [this](const std_msgs::Float32::ConstPtr &msg){
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        const uint32_t state = emergencyStopState_.load(std::memory_order_acquire);
        if ((state & 1U) != 0U) {
          // Bind reset to the exact emergency generation observed here.
          emergencyStopResetTarget_.store(state, std::memory_order_release);
        }
      }
    };
    emgStopResetSub_ = controllerNH.subscribe<std_msgs::Float32>(
        "/reset_emergency_stop", 1, emergencyStopResetCallback);

    // start control
    auto startControlCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::START);
      }
    };
    startCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/start_control", 1, startControlCallback);

    auto shutdownControlCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      (void)msg;
      shutdownControlRequested_.store(true, std::memory_order_release);
    };
    shutdownCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/shut_down", 1, shutdownControlCallback);

    // switchMode (LIE <-> STAND)
    auto switchModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::SWITCH_MODE);
      }
    };
    switchModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/switch_mode", 1, switchModeCallback);

    auto lie2standCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::LIE_TO_STAND);
      }
    };
    lie2standCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/lie2stand", 1, lie2standCallback);

    auto stand2lieCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::STAND_TO_LIE);
      }
    };
    stand2lieCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/stand2lie", 1, stand2lieCallback);

    // walkMode
    auto walkModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::WALK);
      }
    };
    walkModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/walk_mode", 1, walkModeCallback);

    auto walk2standCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::WALK_TO_STAND);
      }
    };
    walk2standSub_ = controllerNH.subscribe<std_msgs::Float32>("/walk2stand", 1, walk2standCallback);

    // danceMode
    auto danceModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::DANCE);
      }
    };
    danceModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/dance_mode", 1, danceModeCallback);

    auto downModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::DOWN);
      }
    };
    downModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/down_mode", 1, downModeCallback);

    auto upModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::UP);
      }
    };
    upModeSub_ = controllerNH.subscribe<std_msgs::Float32>("/up_mode", 1, upModeCallback);

    // positionMode
    auto positionModeCallback = [this](const std_msgs::Float32::ConstPtr &msg)
    {
      if (std::isfinite(msg->data) && msg->data > 0.0F) {
        enqueueControlRequest(ControlRequestId::POSITION);
      }
    };
    positionCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/position_control", 1, positionModeCallback);

    analysisPublisherRunning_.store(true, std::memory_order_release);
    analysisPublisherThread_ = std::thread(&RLControllerBase::analysisPublisherLoop, this);

    return true;
  }

  std::atomic<scalar_t> kp_stance{0};
  std::atomic<scalar_t> kd_stance{3};

  void RLControllerBase::starting(const ros::Time &time)
  {
    loopCount_ = 0;
    start_control.store(false, std::memory_order_release);
    clearPendingControlRequests();
    emergencyStopResetTarget_.store(0, std::memory_order_release);
    emergencyStopApplied_ = false;
    joystickConnected_.store(true, std::memory_order_release);
    joystickStatusReceived_.store(false, std::memory_order_release);
    joystickDisconnectApplied_ = false;
    for (auto& receivedNs : requestLastReceivedNs_) {
      receivedNs.store(0, std::memory_order_release);
    }
    updateStateEstimation(time, ros::Duration(0.002));
    standPercent_ = 0;
    standTransitionStartNs_ = 0;
    mode_ = Mode::DEFAULT;

  }

  bool RLControllerBase::shouldPublishAnalysisData() const
  {
    return enableStatePublish_ && dataPublishDecimation_ > 0 && loopCount_ % dataPublishDecimation_ == 0;
  }

  bool RLControllerBase::shouldPublishTf() const
  {
    return enableTfPublish_ && tfPublishDecimation_ > 0 && loopCount_ % tfPublishDecimation_ == 0;
  }

  void RLControllerBase::resizeAnalysisMessages()
  {
    real_joint_pos_msg_.data.resize(joint_dim_);
    real_joint_vel_msg_.data.resize(joint_dim_);
    real_torque_msg_.data.resize(joint_dim_);
    real_imu_angular_vel_msg_.data.resize(3);
    real_imu_linear_acc_msg_.data.resize(3);
    real_imu_euler_xyz_msg_.data.resize(3);
    output_planned_joint_pos_msg_.data.resize(joint_dim_);
    output_planned_joint_vel_msg_.data.resize(joint_dim_);
    output_planned_torque_msg_.data.resize(joint_dim_);
    output_command_kp_msg_.data.resize(joint_dim_);
    output_command_kd_msg_.data.resize(joint_dim_);
    output_command_ff_msg_.data.resize(joint_dim_);
    mode_command_msg_.data.resize(5);
  }

  bool RLControllerBase::acceptControlRequest(ControlRequestId requestId,
                                              int64_t steadyNowNsValue,
                                              int64_t debounceNs)
  {
    const size_t index = static_cast<size_t>(requestId);
    const int64_t lastAcceptedNs = requestLastAcceptedNs_[index];
    if (lastAcceptedNs != 0 && steadyNowNsValue - lastAcceptedNs <= debounceNs) {
      return false;
    }
    requestLastAcceptedNs_[index] = steadyNowNsValue;
    return true;
  }

  void RLControllerBase::enqueueControlRequest(ControlRequestId requestId) noexcept
  {
    const uint32_t index = static_cast<uint32_t>(requestId);
    if (index < static_cast<uint32_t>(ControlRequestId::COUNT)) {
      // joy_teleop/SBUS may publish continuously while a button is held. Treat
      // a stream with no 250 ms release gap as one request, not repeated toggles.
      constexpr int64_t kReleaseGapNs = 250000000LL;
      const int64_t nowNs = steadyNowNs();
      const int64_t previousNs = requestLastReceivedNs_[index].exchange(
          nowNs, std::memory_order_acq_rel);
      if (previousNs > 0 && nowNs >= previousNs && nowNs - previousNs <= kReleaseGapNs) {
        return;
      }
      pendingControlRequests_.fetch_or(uint32_t{1} << index, std::memory_order_release);
    }
  }

  void RLControllerBase::clearPendingControlRequests()
  {
    pendingControlRequests_.store(0, std::memory_order_release);
    shutdownControlRequested_.store(false, std::memory_order_release);
  }

  void RLControllerBase::enqueueControlEvent(ControlEvent event) noexcept
  {
    const uint32_t index = static_cast<uint32_t>(event);
    if (index < 64U) {
      pendingControlEvents_.fetch_or(uint64_t{1} << index, std::memory_order_relaxed);
    }
  }

  void RLControllerBase::latchEmergencyStop() noexcept
  {
    uint32_t observed = emergencyStopState_.load(std::memory_order_relaxed);
    uint32_t desired = 0;
    do {
      // Incrementing by two preserves bit zero as the latch while changing the
      // generation on every trigger, including repeated triggers while latched.
      desired = (observed + 2U) | 1U;
    } while (!emergencyStopState_.compare_exchange_weak(
        observed, desired, std::memory_order_release, std::memory_order_relaxed));
  }

  bool RLControllerBase::emergencyStopLatched() const noexcept
  {
    return (emergencyStopState_.load(std::memory_order_acquire) & 1U) != 0U;
  }

  bool RLControllerBase::resetEmergencyStopIfRequested() noexcept
  {
    const uint32_t target = emergencyStopResetTarget_.exchange(0, std::memory_order_acq_rel);
    if ((target & 1U) == 0U) {
      return false;
    }

    // A concurrent emergency trigger changes the generation and makes this
    // single CAS fail. A stale reset can never clear a later emergency.
    uint32_t observed = target;
    const uint32_t desired = target & ~uint32_t{1};
    return emergencyStopState_.compare_exchange_strong(
        observed, desired, std::memory_order_acq_rel, std::memory_order_acquire);
  }

  void RLControllerBase::captureCurrentJointAngles()
  {
    if (currentJointAngles_.size() != hybridJointHandles_.size()) {
      currentJointAngles_.resize(hybridJointHandles_.size());
    }
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
      currentJointAngles_[i] = hybridJointHandles_[i].getPosition();
    }
  }

  void RLControllerBase::resetStandTransition(int64_t steadyNowNsValue) noexcept
  {
    standTransitionStartNs_ = steadyNowNsValue;
    standPercent_ = 0;
  }

  scalar_t RLControllerBase::standTransitionPercent() noexcept
  {
    const int64_t nowNs = steadyNowNs();
    if (standTransitionStartNs_ <= 0) {
      standTransitionStartNs_ = nowNs;
      standPercent_ = 0;
      return standPercent_;
    }

    const int64_t elapsedNs = nowNs >= standTransitionStartNs_
        ? nowNs - standTransitionStartNs_ : 0;
    if (standTransitionDurationNs_ <= 0 || elapsedNs >= standTransitionDurationNs_) {
      standPercent_ = 1;
    } else {
      standPercent_ = static_cast<scalar_t>(
          static_cast<double>(elapsedNs) /
          static_cast<double>(standTransitionDurationNs_));
    }
    return standPercent_;
  }

  void RLControllerBase::processControlRequests(int64_t steadyNowNsValue)
  {
    // Shutdown is a safety request: it is never debounced and it cancels all
    // lower-priority requests that may already be pending in callback threads.
    if (shutdownControlRequested_.exchange(false, std::memory_order_acq_rel)) {
      start_control.store(false, std::memory_order_release);
      mode_ = Mode::DEFAULT;
      clearPendingControlRequests();
      enqueueControlEvent(ControlEvent::SHUTDOWN_CONTROL);
      return;
    }

    // Take one mailbox snapshot and discard the snapshot after arbitration.
    // Requests arriving after exchange() remain queued for the next cycle.
    const uint32_t pending = pendingControlRequests_.exchange(0, std::memory_order_acq_rel);
    if (pending == 0U) {
      return;
    }
    if ((pending & (pending - 1U)) != 0U) {
      enqueueControlEvent(ControlEvent::CONTROL_REQUEST_CONFLICT);
    }

    struct Candidate
    {
      ControlRequestId id;
      int64_t debounceNs;
    };
    // Fixed safety-oriented priority. START is first because it acts as a stop
    // while control is enabled. Explicit transitions toward safer modes beat
    // requests that increase motion authority.
    const std::array<Candidate, 10> priority{{
        {ControlRequestId::START, 500000000LL},
        {ControlRequestId::STAND_TO_LIE, 50000000LL},
        {ControlRequestId::WALK_TO_STAND, 50000000LL},
        {ControlRequestId::POSITION, 200000000LL},
        {ControlRequestId::SWITCH_MODE, 800000000LL},
        {ControlRequestId::LIE_TO_STAND, 50000000LL},
        {ControlRequestId::WALK, 200000000LL},
        {ControlRequestId::DANCE, 200000000LL},
        {ControlRequestId::DOWN, 200000000LL},
        {ControlRequestId::UP, 200000000LL},
    }};

    ControlRequestId selected = ControlRequestId::COUNT;
    for (const Candidate& candidate : priority) {
      const uint32_t bit = uint32_t{1} << static_cast<uint32_t>(candidate.id);
      if ((pending & bit) != 0U && controlRequestAllowed(candidate.id) &&
          acceptControlRequest(candidate.id, steadyNowNsValue, candidate.debounceNs)) {
        selected = candidate.id;
        break;
      }
    }

    const ControlTransition transition = applyControlRequest(
        mode_, start_control.load(std::memory_order_acquire), selected);
    if (!transition.accepted) return;
    start_control.store(transition.startControl, std::memory_order_release);
    if (transition.resetStandTransition) resetStandTransition(steadyNowNsValue);
    if (transition.captureCurrentJointAngles) captureCurrentJointAngles();
    if (transition.resetObservation) isfirstRecObs_ = true;
    if (transition.resetDanceTime) danceTimeStep = 0;
    mode_ = transition.mode;
    enqueueControlEvent(transition.event);
  }

  bool RLControllerBase::controlRequestAllowed(ControlRequestId /*requestId*/) const noexcept
  {
    return true;
  }

  void RLControllerBase::update(const ros::Time &time, const ros::Duration &period)
  {
    const int64_t steadyNowNsValue = steadyNowNs();

    // Consume all three components as one published snapshot. Keep command_
    // private to this control iteration so callbacks can never expose a mix
    // of fields from two different /cmd_vel messages.
    if (const Command* latestCommand = commandBuffer_.consumeLatest()) {
      command_ = *latestCommand;
    }

    const bool commandFresh = command_.receivedSteadyNs > 0 &&
        steadyNowNsValue >= command_.receivedSteadyNs &&
        steadyNowNsValue - command_.receivedSteadyNs <= cmdVelTimeoutNs_;
    if (!commandFresh) {
      if (cmdVelWasFresh_) {
        enqueueControlEvent(ControlEvent::CMD_VEL_TIMEOUT);
      }
      command_.x = 0.0;
      command_.y = 0.0;
      command_.yaw = 0.0;
      cmdVelWasFresh_ = false;
    } else {
      cmdVelWasFresh_ = true;
    }

    updateStateEstimation(time, period);
    auto applyImmediateStop = [this]() {
      start_control.store(false, std::memory_order_release);
      mode_ = Mode::DEFAULT;
      clearPendingControlRequests();
      command_.x = 0.0;
      command_.y = 0.0;
      command_.yaw = 0.0;
      cmdVelWasFresh_ = false;
    };
    if (emergencyStopLatched()) {
      applyImmediateStop();
      if (!emergencyStopApplied_) {
        emergencyStopApplied_ = true;
        enqueueControlEvent(ControlEvent::EMERGENCY_STOP);
      }
      // Clearing the latch never resumes control. This cycle remains DEFAULT
      // and a separate, later start request is required.
      if (resetEmergencyStopIfRequested()) {
        emergencyStopApplied_ = false;
        clearPendingControlRequests();
        enqueueControlEvent(ControlEvent::EMERGENCY_STOP_RESET);
      }
    } else if (shutdownControlRequested_.exchange(false, std::memory_order_acq_rel)) {
      applyImmediateStop();
      enqueueControlEvent(ControlEvent::SHUTDOWN_CONTROL);
    } else {
      // Do not carry a reset command forward until some future emergency.
      emergencyStopResetTarget_.store(0, std::memory_order_release);
      processControlRequests(steadyNowNsValue);
    }

    const bool joystickDisconnected =
        joystickStatusReceived_.load(std::memory_order_acquire) &&
        !joystickConnected_.load(std::memory_order_acquire);
    if (joystickDisconnected) {
      applyJoystickDisconnect();
    } else {
      joystickDisconnectApplied_ = false;
    }

    if (!dispatchControlMode()) {
      enqueueControlEvent(ControlEvent::UNEXPECTED_MODE);
      mode_ = Mode::DEFAULT;
      handleDefaultMode();
    }

    // Close the race where a stop callback arrives while a mode handler is
    // running (especially during synchronous inference). Replacing the joint
    // commands here prevents that handler's output from reaching write().
    if (emergencyStopLatched()) {
      applyImmediateStop();
      if (!emergencyStopApplied_) {
        emergencyStopApplied_ = true;
        enqueueControlEvent(ControlEvent::EMERGENCY_STOP);
      }
      handleDefaultMode();
    } else if (shutdownControlRequested_.exchange(false, std::memory_order_acq_rel)) {
      applyImmediateStop();
      enqueueControlEvent(ControlEvent::SHUTDOWN_CONTROL);
      handleDefaultMode();
    } else if (!joystickDisconnected &&
               joystickStatusReceived_.load(std::memory_order_acquire) &&
               !joystickConnected_.load(std::memory_order_acquire)) {
      // Close the race where disconnect arrives while a policy handler runs.
      applyJoystickDisconnect();
      handleDefaultMode();
    }

    enqueueAnalysisSample(time);

    loopCount_++;
  }

  bool RLControllerBase::dispatchControlMode()
  {
    using Handler = void (RLControllerBase::*)();
    static_assert(static_cast<size_t>(Mode::UP) == 6,
                  "ControlMode values must stay contiguous for registry lookup");
    // ControlMode values are stable wire-level identifiers. Keeping the
    // handlers in value order gives the RT loop constant-time dispatch.
    static constexpr std::array<Handler, 7> registrations{{
        &RLControllerBase::handleLieMode,
        &RLControllerBase::handleStandMode,
        &RLControllerBase::handleWalkMode,
        &RLControllerBase::handleDanceMode,
        &RLControllerBase::handleDefaultMode,
        &RLControllerBase::handleDownMode,
        &RLControllerBase::handleUpMode,
    }};

    const size_t index = static_cast<size_t>(mode_);
    if (index >= registrations.size()) return false;
    (this->*registrations[index])();
    return true;
  }

  void RLControllerBase::updateStateEstimation(const ros::Time &time, const ros::Duration & /*period*/)
  {
    quaternion_t quat;
    vector3_t angularVel, linearAccel;

    for (size_t i = 0; i < (size_t)actuatedDofNum_; ++i) {
      propri_.jointPos(i) = hybridJointHandles_[i].getPosition();
      propri_.jointVel(i) = hybridJointHandles_[i].getVelocity();
    }
    for (size_t i = 0; i < (size_t)policyDofNum_; ++i) {
      policyPropri_.jointPos(i) = policyJointHandles_[i].getPosition();
      policyPropri_.jointVel(i) = policyJointHandles_[i].getVelocity();
    }
    for (size_t i = 0; i < 4; ++i) quat.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
    for (size_t i = 0; i < 3; ++i) {
      angularVel(i)  = imuSensorHandles_.getAngularVelocity()[i];
      linearAccel(i) = imuSensorHandles_.getLinearAcceleration()[i];
    }
    propri_.baseAngVel  = angularVel;
    propri_.robot_quat_ = quat;

    policyPropri_.baseAngVel  = angularVel;
    policyPropri_.robot_quat_ = quat;

    vector3_t gravityVector(0, 0, -1);
    vector3_t zyx = quatToZyx(quat);
    matrix3_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
    propri_.baseEulerXyz = quatToXyz(quat);
    propri_.projectedGravity = inverseRot * gravityVector;
    policyPropri_.baseEulerXyz = propri_.baseEulerXyz;
    policyPropri_.projectedGravity = propri_.projectedGravity;
    phase_ = time.toSec();

  }

  void RLControllerBase::enqueueAnalysisSample(const ros::Time& time)
  {
    const bool publishState = shouldPublishAnalysisData();
    const bool publishTf = shouldPublishTf() && static_cast<bool>(robotStatePublisherPtr_);
    if (!publishState && !publishTf) return;

    AnalysisSample sample;
    sample.stampSec = time.sec;
    sample.stampNsec = time.nsec;
    sample.dof = hybridJointHandles_.size();
    sample.publishState = publishState;
    sample.publishTf = publishTf;

    for (size_t i = 0; i < sample.dof; ++i) {
      auto& handle = hybridJointHandles_[i];
      sample.jointPos[i] = handle.getPosition();
      sample.jointVel[i] = handle.getVelocity();
      sample.jointTorque[i] = handle.getEffort();
      sample.positionDesired[i] = handle.getPositionDesired();
      sample.velocityDesired[i] = handle.getVelocityDesired();
      sample.kp[i] = handle.getKp();
      sample.kd[i] = handle.getKd();
      sample.feedforward[i] = handle.getFeedforward();
    }
    for (size_t i = 0; i < 4; ++i) {
      sample.orientation[i] = imuSensorHandles_.getOrientation()[i];
    }
    for (size_t i = 0; i < 3; ++i) {
      sample.angularVelocity[i] = imuSensorHandles_.getAngularVelocity()[i];
      sample.linearAcceleration[i] = imuSensorHandles_.getLinearAcceleration()[i];
      sample.eulerXyz[i] = propri_.baseEulerXyz[i];
    }
    sample.modeCommand[0] = static_cast<double>(mode_);
    sample.modeCommand[1] = command_.x;
    sample.modeCommand[2] = command_.y;
    sample.modeCommand[3] = command_.yaw;
    sample.modeCommand[4] = static_cast<double>(loopCount_);

    if (!analysisBuffer_.push(sample)) {
      analysisDropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void RLControllerBase::logPendingControlEvents()
  {
    const uint64_t pending = pendingControlEvents_.exchange(0, std::memory_order_acq_rel);
    for (uint32_t index = 0; index < static_cast<uint32_t>(ControlEvent::COUNT); ++index) {
      if ((pending & (uint64_t{1} << index)) == 0U) continue;
      switch (static_cast<ControlEvent>(index)) {
      case ControlEvent::EMERGENCY_STOP:
        ROS_ERROR("[RLControllerBase] Emergency stop applied before command generation");
        break;
      case ControlEvent::CMD_VEL_TIMEOUT:
        ROS_WARN_THROTTLE(1.0, "[RLControllerBase] /cmd_vel timed out; velocity command cleared");
        break;
      case ControlEvent::JOYSTICK_DISCONNECTED:
        ROS_ERROR("[RLControllerBase] USB joystick disconnected; safety fallback applied");
        break;
      case ControlEvent::START_CONTROL:
        ROS_INFO("Start Control");
        break;
      case ControlEvent::SHUTDOWN_CONTROL:
        ROS_INFO("ShutDown Control");
        break;
      case ControlEvent::STAND_TO_LIE:
        ROS_INFO("STAND2LIE");
        break;
      case ControlEvent::LIE_TO_STAND:
        ROS_INFO("LIE2STAND");
        break;
      case ControlEvent::STAND_TO_WALK:
        ROS_INFO("STAND2WALK");
        break;
      case ControlEvent::DANCE_TO_WALK:
        ROS_INFO("DANCE2WALK");
        break;
      case ControlEvent::DOWN_TO_WALK:
        ROS_INFO("DOWN2WALK");
        break;
      case ControlEvent::UP_TO_WALK:
        ROS_INFO("UP2WALK");
        break;
      case ControlEvent::TO_STAND:
        ROS_INFO("->STAND");
        break;
      case ControlEvent::WALK_TO_DANCE:
        ROS_INFO("WALK2DANCE");
        break;
      case ControlEvent::WALK_TO_DOWN:
        ROS_INFO("WALK2DOWN");
        break;
      case ControlEvent::DOWN_TO_UP:
        ROS_INFO("DOWN_DAMPING2UP");
        break;
      case ControlEvent::DEFAULT_TO_LIE:
        ROS_INFO("DEF2LIE");
        break;
      case ControlEvent::UNEXPECTED_MODE:
        ROS_ERROR_THROTTLE(1.0, "[RLControllerBase] Unexpected mode; forced DEFAULT");
        break;
      case ControlEvent::FALL_PROTECTION:
        ROS_WARN_THROTTLE(1.0, "[AcController] Fall protection triggered, switching to DEFAULT");
        break;
      case ControlEvent::WALK_ACTION_TIMEOUT:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Walk inference action timed out, switching to DEFAULT");
        break;
      case ControlEvent::DANCE_ACTION_TIMEOUT:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Dance inference action timed out, switching to DEFAULT");
        break;
      case ControlEvent::DOWN_ACTION_TIMEOUT:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Down inference action timed out, switching to DEFAULT");
        break;
      case ControlEvent::UP_ACTION_TIMEOUT:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Up inference action timed out, switching to DEFAULT");
        break;
      case ControlEvent::WALK_POLICY_ERROR:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Walk policy inference/output validation failed");
        break;
      case ControlEvent::DANCE_POLICY_ERROR:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Dance policy inference/output validation failed");
        break;
      case ControlEvent::DOWN_POLICY_ERROR:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Down policy inference/output validation failed");
        break;
      case ControlEvent::UP_POLICY_ERROR:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Up policy inference/output validation failed");
        break;
      case ControlEvent::OBSERVATION_BUFFER_OVERFLOW:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Observation exceeds preallocated async buffer");
        break;
      case ControlEvent::ACTION_SIZE_MISMATCH:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Async action size mismatch");
        break;
      case ControlEvent::WALK_PROPRIO_MISMATCH:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Walk proprioception size mismatch");
        break;
      case ControlEvent::DANCE_PROPRIO_MISMATCH:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Dance proprioception/motion size mismatch");
        break;
      case ControlEvent::DOWN_PROPRIO_MISMATCH:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Down proprioception/motion size mismatch");
        break;
      case ControlEvent::UP_PROPRIO_MISMATCH:
        ROS_ERROR_THROTTLE(1.0, "[AcController] Up proprioception/motion size mismatch");
        break;
      case ControlEvent::DANCE_COMPLETE:
        ROS_INFO("DANCE2WALK (complete)");
        break;
      case ControlEvent::DOWN_COMPLETE:
        ROS_INFO("DOWN complete: entering low-damping hold (kp=0, kd=0.1)");
        break;
      case ControlEvent::UP_COMPLETE:
        ROS_INFO("UP2WALK (complete)");
        break;
      case ControlEvent::EMERGENCY_STOP_RESET:
        ROS_WARN("[RLControllerBase] Emergency-stop latch reset; control remains disabled until a new start request");
        break;
      case ControlEvent::CONTROL_REQUEST_CONFLICT:
        ROS_WARN_THROTTLE(1.0, "[RLControllerBase] Concurrent state requests arbitrated; only one request accepted");
        break;
      case ControlEvent::COUNT:
        break;
      }
    }
  }

  void RLControllerBase::analysisPublisherLoop()
  {
    pthread_setname_np(pthread_self(), "rl_state_pub");
    auto publishArray = [](const ros::Publisher& publisher,
                           std_msgs::Float64MultiArray& message,
                           const auto& values,
                           size_t count) {
      if (publisher.getNumSubscribers() == 0) return;
      if (message.data.size() != count) message.data.resize(count);
      for (size_t i = 0; i < count; ++i) message.data[i] = values[i];
      publisher.publish(message);
    };

    while (analysisPublisherRunning_.load(std::memory_order_acquire)) {
      logPendingControlEvents();
      const uint64_t dropped = analysisDropped_.exchange(0, std::memory_order_relaxed);
      if (dropped > 0) {
        ROS_WARN_THROTTLE(1.0, "Dropped %llu controller analysis publication samples",
                          static_cast<unsigned long long>(dropped));
      }
      AnalysisSample sample;
      while (analysisBuffer_.pop(sample)) {
        if (sample.publishState) {
          publishArray(realImuAngularVelPublisher_, real_imu_angular_vel_msg_,
                       sample.angularVelocity, sample.angularVelocity.size());
          publishArray(realImuLinearAccPublisher_, real_imu_linear_acc_msg_,
                       sample.linearAcceleration, sample.linearAcceleration.size());
          publishArray(realImuEulerXyzPulbisher, real_imu_euler_xyz_msg_,
                       sample.eulerXyz, sample.eulerXyz.size());
          publishArray(realTorquePublisher_, real_torque_msg_, sample.jointTorque, sample.dof);
          publishArray(realJointPosPublisher_, real_joint_pos_msg_, sample.jointPos, sample.dof);
          publishArray(realJointVelPublisher_, real_joint_vel_msg_, sample.jointVel, sample.dof);
          publishArray(outputPlannedJointPosPublisher_, output_planned_joint_pos_msg_,
                       sample.positionDesired, sample.dof);
          publishArray(outputPlannedJointVelPublisher_, output_planned_joint_vel_msg_,
                       sample.velocityDesired, sample.dof);
          publishArray(outputCommandKpPublisher_, output_command_kp_msg_, sample.kp, sample.dof);
          publishArray(outputCommandKdPublisher_, output_command_kd_msg_, sample.kd, sample.dof);
          publishArray(outputCommandFfPublisher_, output_command_ff_msg_, sample.feedforward, sample.dof);
          if (outputPlannedTorquePublisher_.getNumSubscribers() > 0) {
            for (size_t i = 0; i < sample.dof; ++i) {
              output_planned_torque_msg_.data[i] = sample.feedforward[i] +
                  sample.kp[i] * (sample.positionDesired[i] - sample.jointPos[i]) +
                  sample.kd[i] * (sample.velocityDesired[i] - sample.jointVel[i]);
            }
            outputPlannedTorquePublisher_.publish(output_planned_torque_msg_);
          }
          publishArray(modeCommandPublisher_, mode_command_msg_,
                       sample.modeCommand, sample.modeCommand.size());
        }

        if (sample.publishTf && robotStatePublisherPtr_) {
          const ros::Time stamp(sample.stampSec, sample.stampNsec);
          if (!fixedTransformsPublished_) {
            robotStatePublisherPtr_->publishFixedTransforms(true);
            fixedTransformsPublished_ = true;
          }
          tf::Transform baseTransform;
          baseTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
          baseTransform.setRotation(tf::Quaternion(
              sample.orientation[0], sample.orientation[1],
              sample.orientation[2], sample.orientation[3]));
          tfBroadcaster_.sendTransform(
              tf::StampedTransform(baseTransform, stamp, "world", "base_link"));
          for (size_t i = 0; i < sample.dof; ++i) {
            joint_positions_msg_[jointNames_[i]] = sample.jointPos[i];
          }
          robotStatePublisherPtr_->publishTransforms(joint_positions_msg_, stamp);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void RLControllerBase::stopAnalysisPublisher()
  {
    analysisPublisherRunning_.store(false, std::memory_order_release);
    if (analysisPublisherThread_.joinable()) {
      analysisPublisherThread_.join();
    }
  }

  void RLControllerBase::cmdVelCallback(const geometry_msgs::Twist &msg)
  {
    Command& nextCommand = commandBuffer_.producerBuffer();
    nextCommand.x = msg.linear.x;
    nextCommand.y = msg.linear.y;
    nextCommand.yaw = msg.angular.z;
    nextCommand.receivedSteadyNs = steadyNowNs();
    commandBuffer_.publish();
  }

  void RLControllerBase::dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level)
  {
    kp_stance = config.kp_stance;
    kd_stance = config.kd_stance;
  }

  void RLControllerBase::joyInfoCallback(const sensor_msgs::Joy &msg)
  {
    if (msg.header.frame_id.empty()) return;

    constexpr size_t axesCapacity = sizeof(joyInfo.axes) / sizeof(joyInfo.axes[0]);
    constexpr size_t buttonsCapacity = sizeof(joyInfo.buttons) / sizeof(joyInfo.buttons[0]);
    const size_t axesCount = std::min(msg.axes.size(), axesCapacity);
    const size_t buttonsCount = std::min(msg.buttons.size(), buttonsCapacity);

    for (size_t i = 0; i < axesCount; ++i) joyInfo.axes[i] = msg.axes[i];
    for (size_t i = axesCount; i < axesCapacity; ++i) joyInfo.axes[i] = 0.0f;

    for (size_t i = 0; i < buttonsCount; ++i) joyInfo.buttons[i] = msg.buttons[i];
    for (size_t i = buttonsCount; i < buttonsCapacity; ++i) joyInfo.buttons[i] = 0;
  }

  void RLControllerBase::joystickConnectedCallback(const std_msgs::Bool &msg) noexcept
  {
    joystickConnected_.store(msg.data, std::memory_order_release);
    joystickStatusReceived_.store(true, std::memory_order_release);
  }

  void RLControllerBase::applyJoystickDisconnect() noexcept
  {
    command_.x = 0.0;
    command_.y = 0.0;
    command_.yaw = 0.0;
    cmdVelWasFresh_ = false;
    pendingControlRequests_.store(0, std::memory_order_release);

    const ControlTransition fallback = joystickDisconnectFallback(
        mode_, start_control.load(std::memory_order_acquire));
    mode_ = fallback.mode;
    start_control.store(fallback.startControl, std::memory_order_release);
    if (fallback.resetObservation) isfirstRecObs_ = true;

    if (!joystickDisconnectApplied_) {
      joystickDisconnectApplied_ = true;
      enqueueControlEvent(ControlEvent::JOYSTICK_DISCONNECTED);
    }
  }

} // namespace legged
