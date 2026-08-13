#pragma once

#include "rl_controllers/LatestTripleBuffer.h"
#include "rl_controllers/ControlStateMachine.h"
#include "rl_controllers/Types.h"
#include <legged_common/SpscRingBuffer.h>
#include <robot_state_publisher/robot_state_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/Joy.h>

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "TutorialsConfig.h"
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/ParamDescription.h>

#include <atomic>
#include <array>
#include <cstdint>
#include <map>
#include <thread>

namespace legged
{

  struct RLRobotCfg
  {
    struct ControlCfg
    {
      float user_torque_limit;
    };

    scalar_t clipActions;
    scalar_t clipObs;

    ControlCfg controlCfg;
  };

  struct JoyInfo
  {
    float axes[8]{};
    int buttons[12]{};
  };

  struct Proprioception
  {
    vector_t jointPos;
    vector_t jointVel;
    vector3_t baseAngVel;
    vector3_t baseEulerXyz;
    vector3_t projectedGravity;
    quaternion_t robot_quat_;
  };

  struct Command
  {
    scalar_t x{0};
    scalar_t y{0};
    scalar_t yaw{0};
    int64_t receivedSteadyNs{0};
  };

  class RLControllerBase : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                                 ContactSensorInterface>
  {
  public:
    using Mode = ControlMode;

    RLControllerBase() = default;
    ~RLControllerBase() override;
    virtual bool init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH);
    virtual void starting(const ros::Time &time);
    virtual void update(const ros::Time &time, const ros::Duration &period);

    virtual bool loadModel(ros::NodeHandle &nh) = 0;
    virtual bool loadRLCfg(ros::NodeHandle &nh) = 0;
    virtual bool loadMotions(ros::NodeHandle &nh) = 0;
    virtual void computeObservation() = 0;

    // Pure virtual mode hooks keep this class an abstract controller contract.
    // The common modes still have reusable definitions in their own .cpp files.
    virtual void handleLieMode() = 0;
    virtual void handleStandMode() = 0;
    virtual void handleDefaultMode() = 0;
    virtual void handleWalkMode() = 0;
    virtual void handleDanceMode() = 0;
    virtual void handleDownMode() = 0;
    virtual void handleUpMode() = 0;

    std::unique_ptr<dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>> server_ptr_;
    void dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level);

  protected:
    using ControlRequestId = ControlRequest;
    static_assert(static_cast<size_t>(ControlRequestId::COUNT) <= 32,
                  "Control requests must fit in the lock-free request mailbox");

    static_assert(static_cast<uint8_t>(ControlEvent::COUNT) <= 64,
                  "Control events must fit in the lock-free event mailbox");
    static_assert(std::atomic<uint64_t>::is_always_lock_free,
                  "RT control-event reporting requires lock-free 64-bit atomics");

    virtual void updateStateEstimation(const ros::Time &time, const ros::Duration &period);
    virtual void cmdVelCallback(const geometry_msgs::Twist &msg);
    virtual void joyInfoCallback(const sensor_msgs::Joy &msg);
    void joystickConnectedCallback(const std_msgs::Bool &msg) noexcept;
    void applyJoystickDisconnect() noexcept;
    void processControlRequests(int64_t steadyNowNs);
    virtual bool controlRequestAllowed(ControlRequestId requestId) const noexcept;
    bool acceptControlRequest(ControlRequestId requestId,
                              int64_t steadyNowNs,
                              int64_t debounceNs);
    void enqueueControlRequest(ControlRequestId requestId) noexcept;
    void clearPendingControlRequests();
    void enqueueControlEvent(ControlEvent event) noexcept;
    void latchEmergencyStop() noexcept;
    bool emergencyStopLatched() const noexcept;
    bool resetEmergencyStopIfRequested() noexcept;
    void captureCurrentJointAngles();
    void resetStandTransition(int64_t steadyNowNs) noexcept;
    scalar_t standTransitionPercent() noexcept;
    bool dispatchControlMode();
    bool shouldPublishAnalysisData() const;
    bool shouldPublishTf() const;
    void resizeAnalysisMessages();
    bool isfirstRecObs_{true};

    Mode mode_{Mode::DEFAULT};
    int64_t loopCount_{0};
    // /cmd_vel has one serialized ROS subscription callback (producer) and
    // the RT update loop is the sole consumer.
    LatestTripleBuffer<Command> commandBuffer_;
    Command command_;
    RLRobotCfg robotCfg_{};

    std::map<std::string, scalar_t> standjointState_{
      {"l_leg_hip_pitch_joint", -0.1495}, {"r_leg_hip_pitch_joint", -0.1495},
      {"l_leg_hip_roll_joint", 0.0}, {"r_leg_hip_roll_joint", 0.0},
      {"l_leg_hip_yaw_joint", 0.0}, {"r_leg_hip_yaw_joint", 0.0},
      {"l_leg_knee_joint", 0.3215}, {"r_leg_knee_joint", 0.3215},
      {"l_leg_ankle_pitch_joint", -0.1720}, {"r_leg_ankle_pitch_joint", -0.1720},
      {"l_leg_ankle_roll_joint", 0.0}, {"r_leg_ankle_roll_joint", 0.0},
      {"waist_yaw_joint", 0.0}, {"waist_roll_joint", 0.0}, {"waist_pitch_joint", 0.0},
      {"l_arm_shoulder_pitch_joint", 0.0}, {"r_arm_shoulder_pitch_joint", 0.0},
      {"l_arm_shoulder_roll_joint", 0.2618}, {"r_arm_shoulder_roll_joint", -0.2618},
      {"l_arm_shoulder_yaw_joint", 0.0}, {"r_arm_shoulder_yaw_joint", 0.0},
      {"l_arm_elbow_joint", 0.0}, {"r_arm_elbow_joint", 0.0}
    };

    std::map<std::string, scalar_t> liejointState_{
      {"l_leg_hip_pitch_joint", 0.0}, {"r_leg_hip_pitch_joint", 0.0},
      {"l_leg_hip_roll_joint", 0.0}, {"r_leg_hip_roll_joint", 0.0},
      {"l_leg_hip_yaw_joint", 0.0}, {"r_leg_hip_yaw_joint", 0.0},
      {"l_leg_knee_joint", 0.0}, {"r_leg_knee_joint", 0.0},
      {"l_leg_ankle_pitch_joint", 0.0}, {"r_leg_ankle_pitch_joint", 0.0},
      {"l_leg_ankle_roll_joint", 0.0}, {"r_leg_ankle_roll_joint", 0.0},
      {"waist_yaw_joint", 0.0}, {"waist_roll_joint", 0.0}, {"waist_pitch_joint", 0.0},
      {"l_arm_shoulder_pitch_joint", 0.0}, {"r_arm_shoulder_pitch_joint", 0.0},
      {"l_arm_shoulder_roll_joint", 0.0}, {"r_arm_shoulder_roll_joint", 0.0},
      {"l_arm_shoulder_yaw_joint", 0.0}, {"r_arm_shoulder_yaw_joint", 0.0},
      {"l_arm_elbow_joint", 0.0}, {"r_arm_elbow_joint", 0.0}
    };

    JoyInfo joyInfo;
    std::atomic_bool start_control{false};
    std::atomic_bool position_control{false};
    // Bit zero is the latched state. Bits 1..31 form a generation counter so
    // reset cannot erase an emergency-stop trigger racing with it.
    std::atomic<uint32_t> emergencyStopState_{0};
    std::atomic<uint32_t> emergencyStopResetTarget_{0};
    bool emergencyStopApplied_{false};
    std::atomic<uint32_t> pendingControlRequests_{0};
    std::atomic_bool shutdownControlRequested_{false};
    std::array<std::atomic<int64_t>, static_cast<size_t>(ControlRequestId::COUNT)> requestLastReceivedNs_;
    std::array<int64_t, static_cast<size_t>(ControlRequestId::COUNT)> requestLastAcceptedNs_{};
    int64_t cmdVelTimeoutNs_{200000000LL};
    bool cmdVelWasFresh_{false};
    std::atomic_bool joystickConnected_{true};
    std::atomic_bool joystickStatusReceived_{false};
    bool joystickDisconnectApplied_{false};

    vector_t rbdState_;
    vector_t measuredRbdState_;
    Proprioception propri_;
    Proprioception policyPropri_;

    std::vector<HybridJointHandle> hybridJointHandles_;
    std::vector<HybridJointHandle> policyJointHandles_;
    std::vector<std::string> jointNames_;
    std::vector<std::string> policyJointNames_;

    hardware_interface::ImuSensorHandle imuSensorHandles_;
    std::vector<ContactSensorHandle> contactHandles_;

    ros::Subscriber cmdVelSub_;
    ros::Subscriber joyCmdVelSub_;
    ros::Subscriber joyInfoSub_;
    ros::Subscriber joystickConnectedSub_;
    ros::Subscriber emgStopSub_;
    ros::Subscriber emgStopResetSub_;
    ros::Subscriber startCtrlSub_;
    ros::Subscriber shutdownCtrlSub_;
    ros::Subscriber switchModeSub_;
    ros::Subscriber lie2standCtrlSub_;
    ros::Subscriber stand2lieCtrlSub_;
    ros::Subscriber walkModeSub_;
    ros::Subscriber walk2standSub_;
    ros::Subscriber danceModeSub_;
    ros::Subscriber downModeSub_;
    ros::Subscriber upModeSub_;

    ros::Subscriber positionCtrlSub_;
    controller_manager_msgs::SwitchController switchCtrlSrv_;
    ros::ServiceClient switchCtrlClient_;

    int actuatedDofNum_ = 23;
    int policyDofNum_ = 23;

    ros::Publisher realJointVelPublisher_;
    ros::Publisher realJointPosPublisher_;
    ros::Publisher realTorquePublisher_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    ros::Publisher realImuAngularVelPublisher_;
    ros::Publisher realImuLinearAccPublisher_;
    ros::Publisher realImuEulerXyzPulbisher;

    ros::Publisher outputPlannedJointVelPublisher_;
    ros::Publisher outputPlannedJointPosPublisher_;
    ros::Publisher outputPlannedTorquePublisher_;
    ros::Publisher outputCommandKpPublisher_;
    ros::Publisher outputCommandKdPublisher_;
    ros::Publisher outputCommandFfPublisher_;
    ros::Publisher modeCommandPublisher_;

    int walkCount_ = 0;
    int danceTimeStep = 0;
    double pd_scale = 1.0;
    float cfg_kd;
    double phase_;
    bool enableStatePublish_{true};
    bool enableTfPublish_{true};
    bool fixedTransformsPublished_{false};
    int dataPublishDecimation_{10};
    int tfPublishDecimation_{10};

  private:
    static constexpr size_t kMaxAnalysisDof = 24;
    struct AnalysisSample
    {
      uint32_t stampSec{0};
      uint32_t stampNsec{0};
      size_t dof{0};
      bool publishState{false};
      bool publishTf{false};
      std::array<double, kMaxAnalysisDof> jointPos{};
      std::array<double, kMaxAnalysisDof> jointVel{};
      std::array<double, kMaxAnalysisDof> jointTorque{};
      std::array<double, kMaxAnalysisDof> positionDesired{};
      std::array<double, kMaxAnalysisDof> velocityDesired{};
      std::array<double, kMaxAnalysisDof> kp{};
      std::array<double, kMaxAnalysisDof> kd{};
      std::array<double, kMaxAnalysisDof> feedforward{};
      std::array<double, 4> orientation{};
      std::array<double, 3> angularVelocity{};
      std::array<double, 3> linearAcceleration{};
      std::array<double, 3> eulerXyz{};
      std::array<double, 5> modeCommand{};
    };

    void enqueueAnalysisSample(const ros::Time& time);
    void analysisPublisherLoop();
    void stopAnalysisPublisher();
    void logPendingControlEvents();

    size_t joint_dim_{0};
    std::vector<scalar_t> currentJointAngles_;
    vector_t standJointAngles_;
    vector_t lieJointAngles_;
    scalar_t standPercent_{0};
    int64_t standTransitionStartNs_{0};
    int64_t standTransitionDurationNs_{2000000000LL};
    std_msgs::Float64MultiArray real_joint_pos_msg_;
    std_msgs::Float64MultiArray real_joint_vel_msg_;
    std_msgs::Float64MultiArray real_torque_msg_;
    std_msgs::Float64MultiArray real_imu_angular_vel_msg_;
    std_msgs::Float64MultiArray real_imu_linear_acc_msg_;
    std_msgs::Float64MultiArray real_imu_euler_xyz_msg_;
    std_msgs::Float64MultiArray output_planned_joint_pos_msg_;
    std_msgs::Float64MultiArray output_planned_joint_vel_msg_;
    std_msgs::Float64MultiArray output_planned_torque_msg_;
    std_msgs::Float64MultiArray output_command_kp_msg_;
    std_msgs::Float64MultiArray output_command_kd_msg_;
    std_msgs::Float64MultiArray output_command_ff_msg_;
    std_msgs::Float64MultiArray mode_command_msg_;
    std::map<std::string, scalar_t> joint_positions_msg_;
    tf::TransformBroadcaster tfBroadcaster_;
    SpscRingBuffer<AnalysisSample, 128> analysisBuffer_;
    std::atomic<uint64_t> analysisDropped_{0};
    std::atomic<uint64_t> pendingControlEvents_{0};
    std::atomic_bool analysisPublisherRunning_{false};
    std::thread analysisPublisherThread_;
  };

} // namespace legged
