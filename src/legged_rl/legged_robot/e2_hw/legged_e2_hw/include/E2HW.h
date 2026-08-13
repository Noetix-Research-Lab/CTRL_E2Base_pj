
//
// Created by hang on 5/10/24.
//

#pragma once

//legged_base
#include <legged_hw/LeggedHW.h>
#include <legged_common/SpscRingBuffer.h>
#include "CanErrorDetector.h"
#include "E2JointMap.h"

// standard include
#include <iostream>
#include <fstream>
#include <ostream>
#include <memory.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <time.h>
#include <iomanip>
#include <array>
#include <atomic>
#include <thread>
// msgs
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <controller_manager_msgs/SwitchController.h>
// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// gaoqing_hw.h
#include <cstdio>
// #include "Console.hpp"
// #include "command.h"
// #include "transmit.h"
#include "Types.h"

// e2 Host Computer Program include
#include "utilities.h"
#include "RemoteUserParameter.h"
#include "EthercatParameter.h"
#include "rt_ethercat_config.h"
#include "ControlFSMData.h"
#include "can_protocol_func.h"
#include "orientation_tools.h"

#include "Timer.h"
// #include "MotorConfig.h"
#include "parallel_solve/include_parallel_solve.h"

namespace legged
{

  struct EthercatOptions {
    std::string net_card;
    int ctr_freq;
    int robot_type;
    std::vector<int> node_motor_num;
    std::vector<int> motor_max_torque_rec;
    std::vector<int> motor_min_torque_rec;
    std::vector<int> motor_max_torque_send;
    std::vector<int> motor_min_torque_send;
    std::vector<int> motor_max_current;
    std::vector<int> motor_min_current;
    std::vector<int> motor_max_velocity;
    std::vector<int> motor_min_velocity;
    std::vector<int> right_arm_motor_type;
    std::vector<int> left_arm_motor_type;
    std::vector<int> right_leg_motor_type;
    std::vector<int> left_leg_motor_type;
    std::vector<int> waist_motor_type;
    std::vector<int> right_leg_motor_dir;
    std::vector<int> left_leg_motor_dir;
    std::vector<int> right_arm_motor_dir;
    std::vector<int> left_arm_motor_dir;
    std::vector<int> waist_motor_dir;
  };

  

  const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

  struct E2MotorData
  {
    double pos_, vel_, tau_;                  // state
    double pos_des_, vel_des_, kp_, kd_, ff_; // command
  };

  struct E2ImuData
  {
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
  };

  class E2HW : public LeggedHW
  {
  public:
    E2HW()
    {
    }
    ~E2HW() override;

    /** \brief Get necessary params from param server. Init hardware_interface.
     *
     * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
     * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
     *
     * @param root_nh Root node-handle of a ROS node.
     * @param robot_hw_nh Node-handle for robot hardware.
     * @return True when init successful, False when failed.
     */
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    /** \brief Communicate with hardware. Get data, status of robot.
     *
     * Call @ref Gsmp_LEGGED_SDK::UDP::Recv() to get robot's state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /** \brief Comunicate with hardware. Publish command to robot.
     *
     * Propagate joint state to actuator state for the stored
     * transmission. Limit cmd_effort into suitable value. Call @ref Gsmp_LEGGED_SDK::UDP::Recv(). Publish actuator
     * current state.
     *
     * @param time Current time
     * @param period Current time - last time
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

    void parallel_ankle_solve_state(E2MotorData joint_data[]);
    void parallel_ankle_solve_cmd(E2MotorData joint_data[]);
    void parallel_waist_solve_state(E2MotorData joint_data[]);
    void parallel_waist_solve_cmd(E2MotorData joint_data[]);

  private:
    struct AnkleDebugSample
    {
      std::array<double, 52> values{};
    };

    struct MotorFeedbackSample
    {
      std::array<double, 23> position{};
      std::array<double, 23> velocity{};
      std::array<double, 23> torque{};
      uint32_t stampSec{0};
      uint32_t stampNsec{0};
      uint64_t cycleSequence{0};
      uint64_t sourceCycleSequence{0};
      uint64_t lastInvalidCycleSequence{0};
      uint64_t monotonicTimestampNs{0};
      uint64_t dataAgeNs{0};
      uint64_t totalInvalidCycles{0};
      uint32_t consecutiveInvalidCycles{0};
      int32_t actualWkc{0};
      int32_t expectedWkc{0};
      EthercatCycleState cycleState{EthercatCycleState::Uninitialized};
      bool dataValid{false};
      bool hasValidData{false};
      bool faultLatched{false};
      bool validityTransition{false};
    };

    void motorFeedbackPublisherLoop();
    void ankleDebugLoggerLoop();
    void enqueueMotorFeedbackSample(const ros::Time& time,
                                    const EthercatCycleSnapshot& cycle) noexcept;
    bool setupJoints();

    bool setupImu();

    bool setupContactSensor(ros::NodeHandle &nh);

    void CalibrateImu(const bool is_sim);

    std::shared_ptr<RobotModel> robot_model;
    std::shared_ptr<Frame> frame_ptr;
    std::shared_ptr<ControlFSMData> control_fsm_data;
    
    EthercatOptions config;
    E2MotorData joint_data_[23]{};
    E2ImuData imu_data_{};
    bool contact_state_[4]{};
    int contact_threshold_{};
    
    uint64_t hs = 0;
    Byte8 byte_8;
    Medulla* node_1{nullptr};
    Medulla* node_2{nullptr};
    Medulla* node_3{nullptr};
    ImuRc* imu_rc{nullptr};
    
    vector_t motor_pos_feedback_{23};
    vector_t motor_vel_feedback_{23};
    vector_t motor_tau_feedback_{23};
    vector_t joint_planned_torque_{23};
    ros::Subscriber odom_sub_;
    ros::Publisher motorPosPublisher_;
    ros::Publisher motorVelPublisher_;
    ros::Publisher motorTorquePublisher_;
    ros::Publisher motorFeedbackStampedPublisher_;
    OrientationTools ori_tools_;

    tf::TransformListener *listener_{nullptr};

    // Default values are overwritten from ethercat_config.yaml during init.
    float bias_motor[23]{}; // T-pose

    // The RT loop builds one fixed-size row and pushes it to an SPSC queue.
    // CSV formatting and file I/O are owned by ankleDebugLoggerThread_.
    bool enable_ankle_log_{false};
    std::string log_path_{"/home/noetix/work/legged_e2/src/legged_rl/legged_robot/e2_hw/legged_e2_hw/log_file/log.csv"};
    AnkleDebugSample pendingAnkleDebugSample_;
    bool ankleDebugSamplePending_{false};
    SpscRingBuffer<AnkleDebugSample, 2048> ankleDebugBuffer_;
    std::atomic<uint64_t> ankleDebugDropped_{0};
    std::atomic_bool ankleDebugLoggerRunning_{false};
    std::thread ankleDebugLoggerThread_;

    // The RT read() loop only copies fixed-size samples into this SPSC queue.
    // ROS serialization and publication run in motorFeedbackPublisherThread_.
    int data_publish_decimation_{10};
    uint64_t read_publish_counter_{0};
    bool queued_feedback_validity_initialized_{false};
    bool last_queued_feedback_valid_{false};
    SpscRingBuffer<MotorFeedbackSample, 256> motorFeedbackBuffer_;
    std::atomic<uint64_t> motorFeedbackDropped_{0};
    std::atomic_bool motorFeedbackPublisherRunning_{false};
    std::thread motorFeedbackPublisherThread_;

    // RT producer / non-RT consumer. No ROS calls or dynamic allocation occur
    // in the CAN detector's read() path.
    CanErrorDetector canErrorDetector_;
    CanFaultLatch canFaultLatch_;
    SpscRingBuffer<CanEvent, 32> canEventBuffer_;
    std::atomic<uint32_t> canEventDropped_{0};
    bool canErrorDetectionEnabled_{true};
    std::ofstream canEventLogFile_;

    // Process-lifetime latch. The RT thread only updates atomics; diagnostic
    // output is emitted by motorFeedbackPublisherThread_.
    std::atomic_bool nonFiniteCommandFaultLatched_{false};
    std::atomic_bool nonFiniteCommandFaultEvent_{false};
    std::atomic<int> nonFiniteCommandFaultLocation_{-1};

    bool ankle_motor_ready_flag_{false};
    bool waist_motor_ready_flag_{false};

    // Joint command handles cached once in init() so read() does not re-query
    // them by name (vector<string> alloc + 23 map lookups) every RT cycle.
    std::vector<HybridJointHandle> hybridJointHandlesCache_;
  };
} // namespace legged
