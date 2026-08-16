#include "E2HW.h"
#include <legged_e2_hw/MotorFeedbackStamped.h>
#include <ros/package.h>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <pthread.h>
#include <sstream>

double global_time = 0.0;
// Medulla* node_1;
// Medulla* node_2;
// Medulla* node_3;
// Medulla* node_4;
// ImuRc* imu_rc;

namespace legged
{

  E2HW::~E2HW()
  {
    ankleDebugLoggerRunning_.store(false, std::memory_order_release);
    if (ankleDebugLoggerThread_.joinable()) {
      ankleDebugLoggerThread_.join();
    }
    motorFeedbackPublisherRunning_.store(false, std::memory_order_release);
    if (motorFeedbackPublisherThread_.joinable()) {
      motorFeedbackPublisherThread_.join();
    }
    if (canEventLogFile_.is_open()) {
      canEventLogFile_.close();
    }
    rt_ethercat_shutdown();
    delete listener_;
    listener_ = nullptr;
    std::cout << "~E2HW_END" << std::endl;
  }

  namespace
  {
    constexpr const char* kAnkleDebugCsvHeader =
        "mea_pre_mL4_pos,mea_pre_mL5_pos,mea_pre_mR4_pos,mea_pre_mR5_pos,"
        "mea_pre_mL4_vel,mea_pre_mL5_vel,mea_pre_mR4_vel,mea_pre_mR5_vel,"
        "mea_pre_mL4_ff,mea_pre_mL5_ff,mea_pre_mR4_ff,mea_pre_mR5_ff,"
        "mea_post_jL4_pos,mea_post_jL5_pos,mea_post_jR4_pos,mea_post_jR5_pos,"
        "mea_post_jL4_vel,mea_post_jL5_vel,mea_post_jR4_vel,mea_post_jR5_vel,"
        "mea_post_jL4_ff,mea_post_jL5_ff,mea_post_jR4_ff,mea_post_jR5_ff,"
        "cmd_pre_jL4_pos,cmd_pre_jL5_pos,cmd_pre_jR4_pos,cmd_pre_jR5_pos,"
        "cmd_pre_jL4_vel,cmd_pre_jL5_vel,cmd_pre_jR4_vel,cmd_pre_jR5_vel,"
        "cmd_pre_jL4_pos_des,cmd_pre_jL5_pos_des,cmd_pre_jR4_pos_des,cmd_pre_jR5_pos_des,"
        "cmd_pre_jL4_vel_des,cmd_pre_jL5_vel_des,cmd_pre_jR4_vel_des,cmd_pre_jR5_vel_des,"
        "cmd_pre_jL4_kp,cmd_pre_jL5_kp,cmd_pre_jR4_kp,cmd_pre_jR5_kp,"
        "cmd_pre_jL4_kd,cmd_pre_jL5_kd,cmd_pre_jR4_kd,cmd_pre_jR5_kd,"
        "cmd_post_mL4_ff,cmd_post_mL5_ff,cmd_post_mR4_ff,cmd_post_mR5_ff";

    const char* CanChannelName(CanChannelId channel) noexcept
    {
      static constexpr const char* kNames[CanErrorDetector::kChannelCount] = {
          "node1/can1", "node1/can2", "node1/can3",
          "node2/can1", "node2/can2", "node2/can3",
          "node3/can1", "node3/can2", "node3/can3"};
      const std::size_t index = static_cast<std::size_t>(channel);
      return index < CanErrorDetector::kChannelCount ? kNames[index] : "unknown";
    }

    std::string ExpandLogTimestamp(const std::string &path)
    {
      const std::string placeholder = "{timestamp}";
      const std::size_t position = path.find(placeholder);
      if (position == std::string::npos)
      {
        return path;
      }

      const std::time_t now = std::time(nullptr);
      std::tm local_time;
      localtime_r(&now, &local_time);
      std::ostringstream timestamp;
      timestamp << std::put_time(&local_time, "%y-%m-%d-%H-%M-%S");

      std::string expanded_path = path;
      expanded_path.replace(position, placeholder.size(), timestamp.str());
      return expanded_path;
    }

    bool ValidateEthercatOptions(const EthercatOptionsFromYaml& config)
    {
      auto requireSize = [](const char* name, size_t actual, size_t expected) {
        if (actual == expected) return true;
        ROS_ERROR("[E2HW] Invalid %s size: expected %zu, got %zu", name, expected, actual);
        return false;
      };

      if (config.net_card.empty()) {
        ROS_ERROR("[E2HW] EtherCAT net_card must not be empty");
        return false;
      }
      if (config.ctr_freq <= 0) {
        ROS_ERROR("[E2HW] EtherCAT ctr_freq must be positive, got %d", config.ctr_freq);
        return false;
      }
      if (!requireSize("node_motor_num", config.node_motor_num.size(), 5) ||
          std::any_of(config.node_motor_num.begin(), config.node_motor_num.end(),
                      [](int count) { return count <= 0; }) ||
          std::accumulate(config.node_motor_num.begin(), config.node_motor_num.end(), 0) != 23) {
        ROS_ERROR("[E2HW] node_motor_num must contain five positive entries totaling 23");
        return false;
      }

      struct IntTableRequirement
      {
        const char* name;
        const std::vector<int>* values;
        size_t expectedSize;
      };
      const std::array<IntTableRequirement, 5> motorTypes{{
          {"right_arm_motor_type", &config.right_arm_motor_type, 4},
          {"left_arm_motor_type", &config.left_arm_motor_type, 4},
          {"right_leg_motor_type", &config.right_leg_motor_type, 6},
          {"left_leg_motor_type", &config.left_leg_motor_type, 6},
          {"waist_motor_type", &config.waist_motor_type, 3}}};
      const std::array<IntTableRequirement, 5> motorDirections{{
          {"right_arm_motor_dir", &config.right_arm_motor_dir, 4},
          {"left_arm_motor_dir", &config.left_arm_motor_dir, 4},
          {"right_leg_motor_dir", &config.right_leg_motor_dir, 6},
          {"left_leg_motor_dir", &config.left_leg_motor_dir, 6},
          {"waist_motor_dir", &config.waist_motor_dir, 3}}};

      for (const auto& table : motorTypes) {
        if (!requireSize(table.name, table.values->size(), table.expectedSize)) return false;
      }
      for (const auto& table : motorDirections) {
        if (!requireSize(table.name, table.values->size(), table.expectedSize)) return false;
        if (std::any_of(table.values->begin(), table.values->end(),
                        [](int direction) { return direction != -1 && direction != 1; })) {
          ROS_ERROR("[E2HW] %s entries must be either -1 or 1", table.name);
          return false;
        }
      }

      const size_t limitTableSize = config.motor_max_current.size();
      struct LimitTable
      {
        const char* name;
        const std::vector<double>* values;
      };
      const std::array<LimitTable, 8> limitTables{{
          {"motor_max_torque_rec", &config.motor_max_torque_rec},
          {"motor_min_torque_rec", &config.motor_min_torque_rec},
          {"motor_max_torque_send", &config.motor_max_torque_send},
          {"motor_min_torque_send", &config.motor_min_torque_send},
          {"motor_max_current", &config.motor_max_current},
          {"motor_min_current", &config.motor_min_current},
          {"motor_max_velocity", &config.motor_max_velocity},
          {"motor_min_velocity", &config.motor_min_velocity}}};
      if (limitTableSize == 0) {
        ROS_ERROR("[E2HW] Motor limit tables must not be empty");
        return false;
      }
      for (const auto& table : limitTables) {
        if (!requireSize(table.name, table.values->size(), limitTableSize)) return false;
        if (std::any_of(table.values->begin(), table.values->end(),
                        [](double value) { return !std::isfinite(value); })) {
          ROS_ERROR("[E2HW] %s contains a non-finite value", table.name);
          return false;
        }
      }
      for (const auto& table : motorTypes) {
        for (const int type : *table.values) {
          if (type < 0 || static_cast<size_t>(type) >= limitTableSize) {
            ROS_ERROR("[E2HW] %s contains invalid motor type %d for %zu limit entries",
                      table.name, type, limitTableSize);
            return false;
          }
        }
      }
      for (size_t i = 0; i < limitTableSize; ++i) {
        if (!(config.motor_min_torque_rec[i] < config.motor_max_torque_rec[i]) ||
            !(config.motor_min_torque_send[i] < config.motor_max_torque_send[i]) ||
            !(config.motor_min_current[i] < config.motor_max_current[i]) ||
            !(config.motor_min_velocity[i] < config.motor_max_velocity[i])) {
          ROS_ERROR("[E2HW] Invalid min/max motor limits at type index %zu", i);
          return false;
        }
      }
      if (!requireSize("bias_motor", config.bias_motor.size(), 23)) return false;
      if (std::any_of(config.bias_motor.begin(), config.bias_motor.end(),
                      [](double value) { return !std::isfinite(value); })) {
        ROS_ERROR("[E2HW] bias_motor contains a non-finite value");
        return false;
      }
      return true;
    }
  }

  bool E2HW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {
    bool require_realtime = false;
    bool enable_motor_error_log = true;
    bool enable_check_and_print = false;
    bool enable_motor_non_error_log = false;
    bool enable_thermal_protection_limit_release = true;
    std::string motor_error_log_path = "logs/e2_motor_error.log";
    std::string can_event_log_path = "/home/noetix/{timestamp}_can.log";
    int can_delta_threshold = 200;
    int can_fault_samples = 50;
    int can_recovery_samples = 50;
    int can_enabled_mask = CanErrorDetector::kAllChannelsMask;
    int ethercat_receive_timeout_us = 500;
    robot_hw_nh.param("require_realtime", require_realtime, false);
    robot_hw_nh.param("ethercat_receive_timeout_us", ethercat_receive_timeout_us, 500);
    robot_hw_nh.param("enable_ankle_log", enable_ankle_log_, false);
    robot_hw_nh.param<std::string>("ankle_log_path", log_path_, log_path_);
    root_nh.param("/enable_ankle_log", enable_ankle_log_, enable_ankle_log_);
    root_nh.param<std::string>("/ankle_log_path", log_path_, log_path_);
    robot_hw_nh.param("enable_motor_error_log", enable_motor_error_log, true);
    robot_hw_nh.param("enable_check_and_print", enable_check_and_print, false);
    robot_hw_nh.param("enable_motor_non_error_log", enable_motor_non_error_log, false);
    robot_hw_nh.param("enable_thermal_protection_limit_release", enable_thermal_protection_limit_release, true);
    robot_hw_nh.param<std::string>("motor_error_log_path", motor_error_log_path, motor_error_log_path);
    robot_hw_nh.param("can_error_detection/enabled", canErrorDetectionEnabled_, true);
    robot_hw_nh.param("can_error_detection/delta_threshold", can_delta_threshold, 200);
    robot_hw_nh.param("can_error_detection/fault_consecutive_samples", can_fault_samples, 50);
    robot_hw_nh.param("can_error_detection/recovery_consecutive_samples", can_recovery_samples, 50);
    robot_hw_nh.param("can_error_detection/enabled_mask", can_enabled_mask,
                     static_cast<int>(CanErrorDetector::kAllChannelsMask));
    robot_hw_nh.param<std::string>("can_error_detection/log_path", can_event_log_path,
                                  can_event_log_path);
    root_nh.param("/enable_motor_error_log", enable_motor_error_log, enable_motor_error_log);
    root_nh.param("/enable_check_and_print", enable_check_and_print, enable_check_and_print);
    root_nh.param("/enable_motor_non_error_log", enable_motor_non_error_log, enable_motor_non_error_log);
    root_nh.param("/enable_thermal_protection_limit_release", enable_thermal_protection_limit_release, enable_thermal_protection_limit_release);
    root_nh.param<std::string>("/motor_error_log_path", motor_error_log_path, motor_error_log_path);
    if (ethercat_receive_timeout_us < 50 || ethercat_receive_timeout_us > 1500)
    {
      ROS_ERROR("[E2HW] ethercat_receive_timeout_us must be in [50, 1500], got %d",
                ethercat_receive_timeout_us);
      return false;
    }
    if (can_delta_threshold < 0 || can_fault_samples <= 0 || can_recovery_samples <= 0 ||
        can_enabled_mask < 0 ||
        can_enabled_mask > static_cast<int>(CanErrorDetector::kAllChannelsMask))
    {
      ROS_ERROR("[E2HW] Invalid CAN detector parameters: threshold=%d fault_samples=%d "
                "recovery_samples=%d enabled_mask=%d",
                can_delta_threshold, can_fault_samples, can_recovery_samples, can_enabled_mask);
      return false;
    }
    canErrorDetector_.configure(static_cast<uint32_t>(can_delta_threshold),
                                static_cast<uint32_t>(can_fault_samples),
                                static_cast<uint32_t>(can_recovery_samples),
                                static_cast<uint16_t>(can_enabled_mask));
    ROS_INFO("[E2HW] CAN error detection %s: delta>%d, fault=%d samples, "
             "recovery=%d samples, mask=0x%03x",
             canErrorDetectionEnabled_ ? "enabled" : "disabled", can_delta_threshold,
             can_fault_samples, can_recovery_samples, can_enabled_mask);
    if (canErrorDetectionEnabled_ && !can_event_log_path.empty())
    {
      can_event_log_path = ExpandLogTimestamp(can_event_log_path);
      canEventLogFile_.open(can_event_log_path, std::ios::out | std::ios::app);
      if (!canEventLogFile_.is_open())
      {
        ROS_WARN("[E2HW] Failed to open CAN event log: %s", can_event_log_path.c_str());
      }
      else
      {
        if (canEventLogFile_.tellp() == std::streampos(0))
        {
          canEventLogFile_ << "ros_time,cycle_sequence,state,channel,candidate_motors,"
                              "counter,delta,streak\n";
          canEventLogFile_.flush();
        }
        ROS_INFO("[E2HW] CAN event log: %s", can_event_log_path.c_str());
      }
    }
    motor_error_log_path = ExpandLogTimestamp(motor_error_log_path);
    InitMotorErrorLogger(motor_error_log_path);
    SetMotorErrorLoggingEnabled(enable_motor_error_log);
    SetCheckAndPrintEnabled(enable_check_and_print);
    SetMotorNonErrorLoggingEnabled(enable_motor_non_error_log);
    SetThermalProtectionLimitReleaseEnabled(enable_thermal_protection_limit_release);
    if (!enable_motor_error_log)
    {
      ROS_WARN("[E2HW] motor error logging DISABLED.");
    }
    if (!enable_check_and_print)
    {
      ROS_WARN("[E2HW] asynchronous terminal temperature output DISABLED.");
    }
    if (enable_motor_non_error_log)
    {
      ROS_WARN("[E2HW] motor non-error temperature logging ENABLED; log file may grow quickly.");
    }
    if (!enable_thermal_protection_limit_release)
    {
      ROS_WARN("[E2HW] thermal protection limit release DISABLED; INKEX temperature uses protected conversion.");
    }

    //发布话题
    motorPosPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("data_analysis/motor_pos", 1);
    motorVelPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("data_analysis/motor_vel", 1);
    motorTorquePublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("data_analysis/motor_torque", 1);
    motorFeedbackStampedPublisher_ =
        robot_hw_nh.advertise<legged_e2_hw::MotorFeedbackStamped>(
            "data_analysis/motor_feedback", 4);
    motor_pos_feedback_.setZero();
    motor_vel_feedback_.setZero();
    motor_tau_feedback_.setZero();
    joint_planned_torque_.setZero();

    //std::string net_card;
    const std::string package_path = ros::package::getPath("legged_e2_hw");
    if (package_path.empty()) {
        ROS_ERROR("Failed to find ROS package path for legged_e2_hw.");
        return false;
    }
    std::string user_param_path = package_path + "/config/config.yaml";
    std::string ethercat_param_path = package_path + "/config/ethercat_config.yaml";
    std::string pjsol_param_path = package_path + "/config/pjsol_config.yaml";
    // 通过rosparam读取EtherCAT配置选项

/*     // root_nh.getParam("/ethercat_config/net_card", config.net_card);
    // root_nh.getParam("/ethercat_config/ctr_freq", config.ctr_freq);
    // root_nh.getParam("/ethercat_config/robot_type", config.robot_type);
    // root_nh.getParam("/ethercat_config/node_motor_num", config.node_motor_num);
    // root_nh.getParam("/ethercat_config/motor_max_torque_rec", config.motor_max_torque_rec);
    // root_nh.getParam("/ethercat_config/motor_min_torque_rec", config.motor_min_torque_rec);
    // root_nh.getParam("/ethercat_config/motor_max_torque_send", config.motor_max_torque_send);
    // root_nh.getParam("/ethercat_config/motor_min_torque_send", config.motor_min_torque_send);
    // root_nh.getParam("/ethercat_config/motor_max_current", config.motor_max_current);
    // root_nh.getParam("/ethercat_config/motor_min_current", config.motor_min_current);
    // root_nh.getParam("/ethercat_config/right_arm_motor_type", config.right_arm_motor_type);
    // root_nh.getParam("/ethercat_config/left_arm_motor_type", config.left_arm_motor_type);
    // root_nh.getParam("/ethercat_config/right_leg_motor_type", config.right_leg_motor_type);
    // root_nh.getParam("/ethercat_config/left_leg_motor_type", config.left_leg_motor_type);
    // root_nh.getParam("/ethercat_config/right_leg_motor_dir", config.right_leg_motor_dir);
    // root_nh.getParam("/ethercat_config/left_leg_motor_dir", config.left_leg_motor_dir);
    // root_nh.getParam("/ethercat_config/right_arm_motor_dir", config.right_arm_motor_dir);
    // root_nh.getParam("/ethercat_config/left_arm_motor_dir", config.left_arm_motor_dir);
    // std::cout<<"motor_max_current:"<<config.motor_min_current[0]<<std::endl;

    // if (!root_nh.getParam("/ethercat_cofig/net_card", net_card)) {
    //     net_card = "enp3s0";
    //     ROS_WARN("Parameter depth_image_topic not set. Using default: %s", net_card.c_str());
    // }
*/
    // 默认使用包内 config 目录；如有特殊需求，仍可通过 rosparam 覆盖。
    root_nh.param<std::string>("/user_param_path", user_param_path, user_param_path);
    root_nh.param<std::string>("/ethercat_param_path", ethercat_param_path, ethercat_param_path);

    // 从YAML文件加载EtherCAT配置选项
    EthercatOptionsFromYaml ethercat_options_from_yaml;
    try {
        ethercat_options_from_yaml = drake::yaml::LoadYamlFile<EthercatOptionsFromYaml>(ethercat_param_path);
    } catch (const std::exception& exception) {
        ROS_ERROR("[E2HW] Failed to parse EtherCAT config %s: %s",
                  ethercat_param_path.c_str(), exception.what());
        return false;
    }
    if (!ValidateEthercatOptions(ethercat_options_from_yaml)) {
        ROS_ERROR("[E2HW] Refusing to start with invalid EtherCAT config: %s",
                  ethercat_param_path.c_str());
        return false;
    }
    for (size_t i = 0; i < ethercat_options_from_yaml.bias_motor.size(); ++i) {
        bias_motor[i] = static_cast<float>(ethercat_options_from_yaml.bias_motor[i]);
    }

    PjsolOptionsFromYaml pjsol_options_from_yaml;
    try {
        pjsol_options_from_yaml = drake::yaml::LoadYamlFile<PjsolOptionsFromYaml>(pjsol_param_path);
    } catch (const std::exception& exception) {
        ROS_ERROR("[E2HW] Failed to parse pjsol config %s: %s",
                  pjsol_param_path.c_str(), exception.what());
        return false;
    }

    std::cout<<"user_param_path:"<<user_param_path<<std::endl;

    // 创建用户参数对象并从YAML文件初始化
    std::shared_ptr<RemoteUserParameter> user_param = std::make_shared<RemoteUserParameter>();
    try {
        user_param->initializeFromYamlFile(user_param_path);
    } catch(std::exception& e) {
        printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
    }
   
    // 创建机器人模型对象
    robot_model = std::make_shared<RobotModel>();
    std::cout<<"robot_model "<<robot_model<<std::endl;
    
    // 创建帧对象，包含机器人模型和用户参数 
    frame_ptr = std::make_shared<Frame>(robot_model, user_param);
    std::cout<<"frame_ptr "<<frame_ptr<<std::endl;
    
    // 创建控制状态机数据对象并初始化
    control_fsm_data = std::make_shared<ControlFSMData>(frame_ptr);
    std::cout<<"control_fsm_data "<<control_fsm_data<<std::endl;
    // 通过yaml读取电机设置
    // 读取电机设置并配置EtherCAT
    read_motor_setting(ethercat_options_from_yaml, control_fsm_data);
    /**
    这段代码是一个名为 rt_ethercat_config 的函数，主要用于配置以太网通信中的从设备。在这段代码中，它创建了几个不同的从设备（EthercatSlaveBase），
    并将它们添加到一个名为 slave_dict 的字典中。每个从设备都有一个名称和一个编号，通过 std::make_shared 创建了一个指向该从设备的共享指针，并将其添加到 slave_dict 中。
    总体来说，这段代码的作用是初始化和配置多个从设备，使它们准备好进行以太网通信。
    */
    rt_ethercat_config();

    // Validate and cache the fixed software topology before opening the bus.
    // This also keeps dynamic_cast and map access out of the RT read loop.
    const auto node1It = slave_dict.find(0);
    const auto node2It = slave_dict.find(1);
    const auto node3It = slave_dict.find(2);
    const auto imuIt = slave_dict.find(3);
    if (node1It == slave_dict.end() || node2It == slave_dict.end() ||
        node3It == slave_dict.end() || imuIt == slave_dict.end())
    {
      ROS_ERROR("[E2HW] Incomplete EtherCAT software topology; expected indices 0..3.");
      return false;
    }
    node_1 = dynamic_cast<Medulla*>(node1It->second.get());
    node_2 = dynamic_cast<Medulla*>(node2It->second.get());
    node_3 = dynamic_cast<Medulla*>(node3It->second.get());
    imu_rc = dynamic_cast<ImuRc*>(imuIt->second.get());
    if (node_1 == nullptr || node_2 == nullptr || node_3 == nullptr || imu_rc == nullptr)
    {
      ROS_ERROR("[E2HW] EtherCAT software topology has unexpected slave types.");
      return false;
    }

    // 设置控制状态机数据节点
    control_fsm_data->node_1_ = node1It->second;
    control_fsm_data->node_2_ = node2It->second;
    control_fsm_data->node_3_ = node3It->second;

    // 初始化EtherCAT接口
    std::string if_name(ethercat_options_from_yaml.net_card);
    if (!rt_ethercat_init(if_name, ethercat_receive_timeout_us))
    {
      ROS_ERROR("[E2HW] EtherCAT initialization failed on interface '%s'; refusing to start control loop.",
                if_name.c_str());
      return false;
    }

    listener_ = new tf::TransformListener(root_nh, ros::Duration(5.0), true);

    if (!LeggedHW::init(root_nh, robot_hw_nh))
    {
      printf("flase_bc_leggedHW::init\n");
      return false;
    }
    setupJoints();
    setupImu();
    //setupContactSensor(robot_hw_nh);

    if (!setupParallelSolvers(pjsol_options_from_yaml))
    {
      ROS_ERROR("[E2HW] Failed to initialize pjsol parallel solvers");
      return false;
    }

    // Cache hybrid joint handles once so the RT read() loop avoids per-cycle
    // getNames() allocation and repeated name-based handle lookups.
    {
      const std::vector<std::string> names = hybridJointInterface_.getNames();
      hybridJointHandlesCache_.clear();
      hybridJointHandlesCache_.reserve(names.size());
      for (const auto &name : names)
      {
        hybridJointHandlesCache_.push_back(hybridJointInterface_.getHandle(name));
      }
    }


    // Per-cycle ankle logging uses an RT-safe SPSC handoff. The initialization
    // path owns truncation and the header; the worker owns all later file I/O.
    root_nh.param("/data_publish_decimation", data_publish_decimation_, 10);
    if (data_publish_decimation_ < 1)
    {
      data_publish_decimation_ = 1;
    }
    if (enable_ankle_log_)
    {
      log_path_ = ExpandLogTimestamp(log_path_);
      ROS_WARN("[E2HW] asynchronous ankle debug logging ENABLED: %s", log_path_.c_str());
      std::ofstream log_file(log_path_, std::ios::out);
      if (!log_file.is_open())
      {
        ROS_ERROR("[E2HW] Failed to open ankle debug log: %s", log_path_.c_str());
        return false;
      }
      log_file << kAnkleDebugCsvHeader << '\n';
      log_file.flush();
      if (!log_file)
      {
        ROS_ERROR("[E2HW] Failed to write ankle debug log header: %s", log_path_.c_str());
        return false;
      }
      log_file.close();

      ankleDebugLoggerRunning_.store(true, std::memory_order_release);
      try
      {
        ankleDebugLoggerThread_ = std::thread(&E2HW::ankleDebugLoggerLoop, this);
      }
      catch (const std::exception& exception)
      {
        ankleDebugLoggerRunning_.store(false, std::memory_order_release);
        ROS_ERROR("[E2HW] Failed to start ankle debug logger: %s", exception.what());
        return false;
      }
    }

    motorFeedbackPublisherRunning_.store(true, std::memory_order_release);
    motorFeedbackPublisherThread_ = std::thread(&E2HW::motorFeedbackPublisherLoop, this);

    return true;
  }

  void E2HW::motorFeedbackPublisherLoop()
  {
    pthread_setname_np(pthread_self(), "motor_state_pub");
    std_msgs::Float64MultiArray position_msg;
    std_msgs::Float64MultiArray velocity_msg;
    std_msgs::Float64MultiArray torque_msg;
    legged_e2_hw::MotorFeedbackStamped feedback_msg;
    position_msg.data.resize(23);
    velocity_msg.data.resize(23);
    torque_msg.data.resize(23);

    const auto process_can_event = [this](const CanEvent& can_event) {
      const char* const state_name =
          can_event.state == CanState::Fault ? "DISCONNECTED" : "RECONNECTED";
      if (can_event.state == CanState::Fault) {
        ROS_ERROR("[CAN Disconnected] %s candidates=[%s] counter=%u delta=%u "
                  "bad_samples=%u ros_time=%u.%09u cycle=%llu; "
                  "hardware fail-closed latched mask=0x%03x",
                  CanChannelName(can_event.channel),
                  CanChannelCandidateMotors(can_event.channel), can_event.value,
                  can_event.delta, can_event.streak, can_event.stampSec,
                  can_event.stampNsec,
                  static_cast<unsigned long long>(can_event.cycleSequence),
                  static_cast<unsigned int>(canFaultLatch_.latchedMask()));
      } else {
        ROS_WARN("[CAN Reconnected] %s counter=%u delta=%u stable_samples=%u "
                 "ros_time=%u.%09u cycle=%llu; "
                 "fail-closed remains latched until hardware-node restart",
                 CanChannelName(can_event.channel), can_event.value,
                 can_event.delta, can_event.streak, can_event.stampSec,
                 can_event.stampNsec,
                 static_cast<unsigned long long>(can_event.cycleSequence));
      }
      if (canEventLogFile_.is_open()) {
        canEventLogFile_ << can_event.stampSec << '.'
                         << std::setfill('0') << std::setw(9) << can_event.stampNsec
                         << std::setfill(' ')
                         << ',' << can_event.cycleSequence
                         << ',' << state_name
                         << ',' << CanChannelName(can_event.channel)
                         << ',' << CanChannelCandidateMotors(can_event.channel)
                         << ',' << can_event.value
                         << ',' << can_event.delta
                         << ',' << can_event.streak << '\n';
        // Events are rare; flush each transition so a subsequent shutdown
        // does not lose a fault already consumed by this non-RT worker.
        canEventLogFile_.flush();
      }
    };

    ros::Time last_unreachable_warn_left;
    ros::Time last_unreachable_warn_right;
    ros::Time last_unreachable_warn_waist;
    ros::Time last_state_issue_warn_left;
    ros::Time last_state_issue_warn_right;
    ros::Time last_state_issue_warn_waist;
    const auto warn_unreachable_command = [](ParallelJoint& joint, ros::Time& last_warn) {
      double qr = 0.0;
      double qp = 0.0;
      uint64_t count = 0;
      if (!joint.consumeUnreachableCommandEvent(&qr, &qp, &count)) {
        return;
      }
      const ros::Time now = ros::Time::now();
      if (!last_warn.isZero() && (now - last_warn).toSec() < 1.0) {
        return;
      }
      last_warn = now;
      ROS_WARN("[E2HW] %s command pose unreachable: qr=%.4f qp=%.4f count=%llu "
               "(no clamp, PD continues)",
               joint.name().c_str(), qr, qp,
               static_cast<unsigned long long>(count));
    };
    const auto warn_state_issue = [](ParallelJoint& joint, ros::Time& last_warn) {
      ParallelStateIssue issue = ParallelStateIssue::None;
      uint64_t count = 0;
      double q1 = 0.0;
      double q2 = 0.0;
      if (!joint.consumeStateIssueEvent(&issue, &count, &q1, &q2)) {
        return;
      }
      const ros::Time now = ros::Time::now();
      if (!last_warn.isZero() && (now - last_warn).toSec() < 1.0) {
        return;
      }
      last_warn = now;
      ROS_WARN("[E2HW] %s state %s: q1=%.4f q2=%.4f count=%llu",
               joint.name().c_str(), ParallelStateIssueName(issue), q1, q2,
               static_cast<unsigned long long>(count));
    };

    while (motorFeedbackPublisherRunning_.load(std::memory_order_acquire)) {
      if (nonFiniteCommandFaultEvent_.exchange(false, std::memory_order_acq_rel)) {
        const int location = nonFiniteCommandFaultLocation_.load(std::memory_order_acquire);
        ROS_ERROR("[E2HW] Non-finite motor command rejected before PDO encoding at "
                  "command group=%d slot=%d; hardware fail-closed is latched until restart",
                  location >= 0 ? location / 6 : -1,
                  location >= 0 ? location % 6 : -1);
      }
      warn_unreachable_command(ankle_left, last_unreachable_warn_left);
      warn_unreachable_command(ankle_right, last_unreachable_warn_right);
      warn_unreachable_command(waist, last_unreachable_warn_waist);
      warn_state_issue(ankle_left, last_state_issue_warn_left);
      warn_state_issue(ankle_right, last_state_issue_warn_right);
      warn_state_issue(waist, last_state_issue_warn_waist);
      const uint32_t can_events_dropped = canEventDropped_.exchange(0, std::memory_order_relaxed);
      if (can_events_dropped > 0U) {
        ROS_WARN("Dropped %u CAN state transition events", can_events_dropped);
      }
      CanEvent can_event;
      while (canEventBuffer_.pop(can_event)) {
        process_can_event(can_event);
      }
      const uint64_t dropped = motorFeedbackDropped_.exchange(0, std::memory_order_relaxed);
      if (dropped > 0) {
        ROS_WARN_THROTTLE(1.0, "Dropped %llu motor feedback publication samples",
                          static_cast<unsigned long long>(dropped));
      }
      const auto publish_sample = [&](const MotorFeedbackSample& sample) {
        feedback_msg.header.stamp = ros::Time(sample.stampSec, sample.stampNsec);
        feedback_msg.cycle_sequence = sample.cycleSequence;
        feedback_msg.source_cycle_sequence = sample.sourceCycleSequence;
        feedback_msg.last_invalid_cycle_sequence = sample.lastInvalidCycleSequence;
        feedback_msg.monotonic_timestamp_ns = sample.monotonicTimestampNs;
        feedback_msg.data_age_ns = sample.dataAgeNs;
        feedback_msg.total_invalid_cycles = sample.totalInvalidCycles;
        feedback_msg.consecutive_invalid_cycles = sample.consecutiveInvalidCycles;
        feedback_msg.actual_wkc = sample.actualWkc;
        feedback_msg.expected_wkc = sample.expectedWkc;
        feedback_msg.cycle_state = static_cast<uint8_t>(sample.cycleState);
        feedback_msg.data_valid = sample.dataValid;
        feedback_msg.has_valid_data = sample.hasValidData;
        feedback_msg.fault_latched = sample.faultLatched;
        for (size_t i = 0; i < sample.position.size(); ++i) {
          position_msg.data[i] = sample.position[i];
          velocity_msg.data[i] = sample.velocity[i];
          torque_msg.data[i] = sample.torque[i];
          feedback_msg.position[i] = sample.position[i];
          feedback_msg.velocity[i] = sample.velocity[i];
          feedback_msg.torque[i] = sample.torque[i];
        }
        if (motorFeedbackStampedPublisher_.getNumSubscribers() > 0) {
          motorFeedbackStampedPublisher_.publish(feedback_msg);
        }
        // Legacy topics have no validity metadata. Preserve their historical
        // contract by publishing only fresh, valid process data.
        if (sample.dataValid) {
          if (motorTorquePublisher_.getNumSubscribers() > 0) {
            motorTorquePublisher_.publish(torque_msg);
          }
          if (motorPosPublisher_.getNumSubscribers() > 0) {
            motorPosPublisher_.publish(position_msg);
          }
          if (motorVelPublisher_.getNumSubscribers() > 0) {
            motorVelPublisher_.publish(velocity_msg);
          }
        }
      };

      // Preserve every validity transition, but coalesce ordinary samples to
      // the newest one. This prevents a diagnostic subscriber from driving a
      // 500 Hz publication burst when decimation is configured to 1.
      MotorFeedbackSample sample;
      MotorFeedbackSample latest_regular_sample;
      bool have_latest_regular_sample = false;
      while (motorFeedbackBuffer_.pop(sample)) {
        if (sample.validityTransition) {
          have_latest_regular_sample = false;
          publish_sample(sample);
        } else {
          latest_regular_sample = sample;
          have_latest_regular_sample = true;
        }
      }
      if (have_latest_regular_sample) {
        publish_sample(latest_regular_sample);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // The RT loop is stopped before E2HW destruction. Drain every transition
    // already accepted by the SPSC queue before the log file is closed.
    CanEvent pending_can_event;
    while (canEventBuffer_.pop(pending_can_event)) {
      process_can_event(pending_can_event);
    }
    const uint32_t can_events_dropped =
        canEventDropped_.exchange(0, std::memory_order_relaxed);
    if (can_events_dropped > 0U) {
      ROS_WARN("Dropped %u CAN state transition events", can_events_dropped);
    }
  }

  void E2HW::ankleDebugLoggerLoop()
  {
    pthread_setname_np(pthread_self(), "ankle_log");
    std::ofstream log_file(log_path_, std::ios::out | std::ios::app);
    if (!log_file.is_open()) {
      std::cerr << "[E2HW] Failed to reopen ankle debug log: " << log_path_ << '\n';
    }

    auto last_flush = std::chrono::steady_clock::now();
    const auto write_sample = [&log_file](const AnkleDebugSample& sample) {
      if (!log_file.is_open()) return;
      for (std::size_t index = 0; index < sample.values.size(); ++index) {
        if (index != 0U) log_file << ',';
        log_file << sample.values[index];
      }
      log_file << '\n';
    };

    while (ankleDebugLoggerRunning_.load(std::memory_order_acquire)) {
      AnkleDebugSample sample;
      std::size_t batch_size = 0;
      while (batch_size < 128U && ankleDebugBuffer_.pop(sample)) {
        write_sample(sample);
        ++batch_size;
      }

      const uint64_t dropped = ankleDebugDropped_.exchange(0, std::memory_order_relaxed);
      if (dropped > 0U) {
        std::cerr << "[E2HW] Dropped " << dropped
                  << " ankle debug rows because the async queue was full\n";
      }

      const auto now = std::chrono::steady_clock::now();
      if (log_file.is_open() && now - last_flush >= std::chrono::seconds(1)) {
        log_file.flush();
        last_flush = now;
      }
      if (batch_size == 0U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }

    AnkleDebugSample sample;
    while (ankleDebugBuffer_.pop(sample)) {
      write_sample(sample);
    }
    if (log_file.is_open()) log_file.flush();
  }
  template <int row_>

  using Vector = Eigen::Matrix<double, row_, 1>;

  Vector<12> m_q; // motor feedback (prior conversion)
  Vector<12> m_v; // motor feedback (prior conversion)
  Vector<12> m_t; // motor feedback (prior conversion)

  void E2HW::enqueueMotorFeedbackSample(
      const ros::Time& time, const EthercatCycleSnapshot& cycle) noexcept
  {
    const bool validity_changed =
        !queued_feedback_validity_initialized_ ||
        cycle.dataValid != last_queued_feedback_valid_;
    const bool periodic_sample =
        read_publish_counter_++ % static_cast<uint64_t>(data_publish_decimation_) == 0U;
    if (!periodic_sample && !validity_changed)
    {
      return;
    }

    MotorFeedbackSample sample;
    sample.stampSec = time.sec;
    sample.stampNsec = time.nsec;
    sample.cycleSequence = cycle.cycleSequence;
    sample.sourceCycleSequence = cycle.lastValidSequence;
    sample.lastInvalidCycleSequence = cycle.lastInvalidSequence;
    sample.monotonicTimestampNs = cycle.steadyTimestampNs;
    sample.totalInvalidCycles = cycle.totalInvalid;
    sample.consecutiveInvalidCycles = cycle.consecutiveInvalid;
    sample.actualWkc = cycle.actualWkc;
    sample.expectedWkc = cycle.expectedWkc;
    sample.cycleState = cycle.state;
    sample.dataValid = cycle.dataValid;
    sample.hasValidData = cycle.lastValidSequence != 0U;
    sample.faultLatched = cycle.faultLatched;
    sample.validityTransition = validity_changed;
    if (sample.hasValidData &&
        cycle.steadyTimestampNs >= cycle.lastValidSteadyTimestampNs)
    {
      sample.dataAgeNs = cycle.steadyTimestampNs - cycle.lastValidSteadyTimestampNs;
    }

    for (size_t i = 0; i < sample.position.size(); ++i) {
      sample.position[i] = motor_pos_feedback_(i);
      sample.velocity[i] = motor_vel_feedback_(i);
      sample.torque[i] = motor_tau_feedback_(i);
    }
    if (motorFeedbackBuffer_.push(sample)) {
      // Only acknowledge a transition after it is safely queued. If the ring
      // is full, the next RT cycle retries the transition instead of silently
      // losing the invalid/valid edge.
      queued_feedback_validity_initialized_ = true;
      last_queued_feedback_valid_ = cycle.dataValid;
    } else {
      motorFeedbackDropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void E2HW::read(const ros::Time &time, const ros::Duration &period)
  {
    // 增加心跳信号
    ++hs;
    // 更新每个medulla节点的控制字和心跳信号
    control_fsm_data->control_word_ = 1;
    node_1->medulla_cmd_.hs = hs;
    node_1->medulla_cmd_.control_word = control_fsm_data->control_word_;
    node_2->medulla_cmd_.hs = hs;
    node_2->medulla_cmd_.control_word = control_fsm_data->control_word_;
    node_3->medulla_cmd_.hs = hs;
    node_3->medulla_cmd_.control_word = control_fsm_data->control_word_;
    // The WKC describes the process-data image received by the preceding
    // write() call. Never unpack a partial/stale IOmap as fresh feedback.
    const EthercatCycleSnapshot ethercat_cycle = rt_ethercat_cycle_snapshot();
    ankleDebugSamplePending_ = false;
    if (!ethercat_cycle.dataValid)
    {
      enqueueMotorFeedbackSample(time, ethercat_cycle);
      return;
    }
    rt_ethercat_get_data();
    if (canErrorDetectionEnabled_)
    {
      const CanErrorDetector::CounterArray can_counters{{
          node_1->medulla_data_.can1_error_log,
          node_1->medulla_data_.can2_error_log,
          node_1->medulla_data_.can3_error_log,
          node_2->medulla_data_.can1_error_log,
          node_2->medulla_data_.can2_error_log,
          node_2->medulla_data_.can3_error_log,
          node_3->medulla_data_.can1_error_log,
          node_3->medulla_data_.can2_error_log,
          node_3->medulla_data_.can3_error_log}};
      canErrorDetector_.update(can_counters, [this, &time, &ethercat_cycle](const CanEvent& event) noexcept {
        // Latch in the RT producer before handing the event to the logger. The
        // write() call later in this same control cycle therefore overrides the
        // controller command without waiting for a non-RT thread.
        CanEvent timestamped_event = event;
        timestamped_event.stampSec = time.sec;
        timestamped_event.stampNsec = time.nsec;
        timestamped_event.cycleSequence = ethercat_cycle.cycleSequence;
        canFaultLatch_.update(timestamped_event);
        if (!canEventBuffer_.push(timestamped_event))
        {
          canEventDropped_.fetch_add(1U, std::memory_order_relaxed);
        }
      });
    }
    const auto unpack_motor = [this, &time](uint64_t raw, int group, int slot,
                                             int joint, MotorType type) {
      byte_8.udata = raw;
      unpack_pvt_data_ex(byte_8.buffer, control_fsm_data, group, slot, joint,
                         type, time.sec, time.nsec);
    };

    // PDO order from the reference E2 hardware implementation.
    unpack_motor(node_1->medulla_data_.motor_1, 3, 0, 8, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_2, 3, 1, 9, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_3, 3, 2, 10, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_4, 3, 3, 11, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_5, 3, 4, 12, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_6, 3, 5, 13, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_7, 0, 0, 0, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_8, 0, 1, 1, MotorType::INKEXBOT);
    unpack_motor(node_1->medulla_data_.motor_9, 0, 2, 2, MotorType::INKEXBOT);

    unpack_motor(node_2->medulla_data_.motor_1, 2, 0, 14, MotorType::INKEXBOT);
    unpack_motor(node_2->medulla_data_.motor_2, 2, 1, 15, MotorType::INKEXBOT);
    unpack_motor(node_2->medulla_data_.motor_3, 2, 2, 16, MotorType::INKEXBOT);
    unpack_motor(node_2->medulla_data_.motor_4, 2, 3, 17, MotorType::INKEXBOT);
    unpack_motor(node_2->medulla_data_.motor_5, 2, 4, 18, MotorType::INKEXBOT);
    unpack_motor(node_2->medulla_data_.motor_6, 2, 5, 19, MotorType::INKEXBOT);
    unpack_motor(node_2->medulla_data_.motor_7, 0, 3, 3, MotorType::INKEXBOT);

    unpack_motor(node_3->medulla_data_.motor_1, 1, 0, 4, MotorType::INKEXBOT);
    unpack_motor(node_3->medulla_data_.motor_2, 1, 1, 5, MotorType::INKEXBOT);
    unpack_motor(node_3->medulla_data_.motor_3, 1, 2, 6, MotorType::INKEXBOT);
    unpack_motor(node_3->medulla_data_.motor_4, 1, 3, 7, MotorType::INKEXBOT);
    unpack_motor(node_3->medulla_data_.motor_7, 4, 0, 20, MotorType::INKEXBOT);
    unpack_motor(node_3->medulla_data_.motor_8, 4, 1, 21, MotorType::INKEXBOT);
    unpack_motor(node_3->medulla_data_.motor_9, 4, 2, 22, MotorType::INKEXBOT);
    PublishMotorTemperatureSnapshotIfDue(time.sec, time.nsec);

    // 获取全局时间并设置到帧中
    global_time = control_fsm_data->time_; 
    frame_ptr->setGlobalTime(global_time);

    /*
    元生艾欸姆尤
    */ 
    // 从IMU数据中获取四元数
    imu_data_.ori[0] = (double)imu_rc->imu_rc_data_.q1;
    imu_data_.ori[1] = (double)imu_rc->imu_rc_data_.q2; 
    imu_data_.ori[2] = (double)imu_rc->imu_rc_data_.q3;
    imu_data_.ori[3] = (double)imu_rc->imu_rc_data_.q0;
    // 从IMU数据中获取姿态角速率和加速度
    imu_data_.angular_vel[0] = (double)imu_rc->imu_rc_data_.gyr_x;
    imu_data_.angular_vel[1] = (double)imu_rc->imu_rc_data_.gyr_y;
    imu_data_.angular_vel[2] = (double)imu_rc->imu_rc_data_.gyr_z;     
    imu_data_.linear_acc[0] = (double)imu_rc->imu_rc_data_.acc_x;// 线性加速度
    imu_data_.linear_acc[1] = (double)imu_rc->imu_rc_data_.acc_y;
    imu_data_.linear_acc[2] = (double)imu_rc->imu_rc_data_.acc_z;

    // CalibrateImu(false);

    for (const E2JointChannel& channel : kE2JointChannels) {
      E2MotorData& joint = joint_data_[channel.jointIndex];
      joint.pos_ = control_fsm_data->position_act_raw_[channel.controlGroup][channel.groupIndex] +
                   bias_motor[channel.jointIndex];
      joint.vel_ = control_fsm_data->velocity_act_raw_[channel.controlGroup][channel.groupIndex];
      joint.tau_ = control_fsm_data->torque_act_raw_[channel.controlGroup][channel.groupIndex];
    }

    parallel_ankle_solve_state(joint_data_);
    parallel_waist_solve_state(joint_data_);

    for (const E2JointChannel& channel : kE2JointChannels) {
      const E2MotorData& joint = joint_data_[channel.jointIndex];
      motor_pos_feedback_(channel.jointIndex) = joint.pos_;
      motor_vel_feedback_(channel.jointIndex) = joint.vel_;
      motor_tau_feedback_(channel.jointIndex) = joint.tau_;
    }

    // Copy a fixed-size data+validity snapshot only; the non-RT worker owns
    // all ROS serialization and publication.
    enqueueMotorFeedbackSample(time, ethercat_cycle);

    // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
    for (auto &handle : hybridJointHandlesCache_)
    {
      handle.setFeedforward(0.);
      handle.setVelocityDesired(0.);
      handle.setKd(3.1415);
      handle.setKp(0.);
    }
    
  }


  void E2HW::write(const ros::Time &time, const ros::Duration &period)
  {
    /*
      第一步 命令赋值
    */
    // An EtherCAT or downstream CAN fault is latched until process restart.
    // Keep bus ownership in this RT thread and continue sending a bounded
    // damping command to devices that still respond.
    if (!rt_ethercat_last_cycle_valid() || rt_ethercat_faulted() ||
        canFaultLatch_.faulted() ||
        nonFiniteCommandFaultLatched_.load(std::memory_order_acquire)) {
      for (auto& joint : joint_data_) {
        joint.pos_des_ = joint.pos_;
        joint.vel_des_ = 0.0;
        joint.kp_ = 0.0;
        joint.kd_ = 3.1415;
        joint.ff_ = 0.0;
      }
    }

    parallel_ankle_solve_cmd(joint_data_);
    parallel_waist_solve_cmd(joint_data_);

    for (const E2JointChannel& channel : kE2JointChannels) {
      const E2MotorData& joint = joint_data_[channel.jointIndex];
      control_fsm_data->kp_[channel.controlGroup][channel.groupIndex] =
          channel.directPd ? joint.kp_ : 0.0;
      control_fsm_data->kd_[channel.controlGroup][channel.groupIndex] =
          channel.directPd ? joint.kd_ : 0.0;
      control_fsm_data->position_des_[channel.controlGroup][channel.groupIndex] =
          joint.pos_des_ - bias_motor[channel.jointIndex];
      control_fsm_data->velocity_des_[channel.controlGroup][channel.groupIndex] = joint.vel_des_;
      control_fsm_data->torque_des_[channel.controlGroup][channel.groupIndex] = joint.ff_;
    }

    // Validate the complete command image before encoding the first motor. A
    // single bad value makes the whole PDO frame a logical zero-torque frame,
    // avoiding a partially normal / partially safe command image.
    static constexpr std::array<int, 5> kMotorCounts{{4, 4, 6, 6, 3}};
    int invalidLocation = -1;
    for (int group = 0; group < static_cast<int>(kMotorCounts.size()) && invalidLocation < 0; ++group) {
      for (int slot = 0; slot < kMotorCounts[group]; ++slot) {
        PvtCommandValues command{
            static_cast<float>(control_fsm_data->kp_[group][slot]),
            static_cast<float>(control_fsm_data->kd_[group][slot]),
            static_cast<float>(control_fsm_data->position_des_[group][slot]),
            static_cast<float>(control_fsm_data->velocity_des_[group][slot]),
            static_cast<float>(control_fsm_data->torque_des_[group][slot])};
        if (!SanitizePvtCommandValues(command)) {
          invalidLocation = group * 6 + slot;
          break;
        }
      }
    }
    if (invalidLocation >= 0) {
      const bool wasLatched = nonFiniteCommandFaultLatched_.exchange(
          true, std::memory_order_acq_rel);
      if (!wasLatched) {
        nonFiniteCommandFaultLocation_.store(invalidLocation, std::memory_order_release);
        nonFiniteCommandFaultEvent_.store(true, std::memory_order_release);
      }
      for (int group = 0; group < static_cast<int>(kMotorCounts.size()); ++group) {
        for (int slot = 0; slot < kMotorCounts[group]; ++slot) {
          control_fsm_data->kp_[group][slot] = 0.0;
          control_fsm_data->kd_[group][slot] = 0.0;
          control_fsm_data->position_des_[group][slot] = 0.0;
          control_fsm_data->velocity_des_[group][slot] = 0.0;
          control_fsm_data->torque_des_[group][slot] = 0.0;
        }
      }
    }

    /*
      第二部 命令格式转换 将 control_fsm_data 数据存储到 byte_8.buffer
    */
    const auto pack_motor = [this](uint64_t& target, int group, int slot,
                                    int joint, MotorType type) {
      pack_pvt_cmd_ex(byte_8.buffer,
                      static_cast<float>(control_fsm_data->kp_[group][slot]),
                      static_cast<float>(control_fsm_data->kd_[group][slot]),
                      static_cast<float>(control_fsm_data->position_des_[group][slot]),
                      static_cast<float>(control_fsm_data->velocity_des_[group][slot]),
                      static_cast<float>(control_fsm_data->torque_des_[group][slot]),
                      joint, control_fsm_data, type);
      target = byte_8.udata;
    };

    // PDO order from the reference E2 hardware implementation.
    pack_motor(node_1->medulla_cmd_.motor_1, 3, 0, 8, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_2, 3, 1, 9, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_3, 3, 2, 10, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_4, 3, 3, 11, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_5, 3, 4, 12, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_6, 3, 5, 13, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_7, 0, 0, 0, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_8, 0, 1, 1, MotorType::INKEXBOT);
    pack_motor(node_1->medulla_cmd_.motor_9, 0, 2, 2, MotorType::INKEXBOT);

    pack_motor(node_2->medulla_cmd_.motor_1, 2, 0, 14, MotorType::INKEXBOT);
    pack_motor(node_2->medulla_cmd_.motor_2, 2, 1, 15, MotorType::INKEXBOT);
    pack_motor(node_2->medulla_cmd_.motor_3, 2, 2, 16, MotorType::INKEXBOT);
    pack_motor(node_2->medulla_cmd_.motor_4, 2, 3, 17, MotorType::INKEXBOT);
    pack_motor(node_2->medulla_cmd_.motor_5, 2, 4, 18, MotorType::INKEXBOT);
    pack_motor(node_2->medulla_cmd_.motor_6, 2, 5, 19, MotorType::INKEXBOT);
    pack_motor(node_2->medulla_cmd_.motor_7, 0, 3, 3, MotorType::INKEXBOT);
    node_2->medulla_cmd_.motor_8 = 0;
    node_2->medulla_cmd_.motor_9 = 0;

    pack_motor(node_3->medulla_cmd_.motor_1, 1, 0, 4, MotorType::INKEXBOT);
    pack_motor(node_3->medulla_cmd_.motor_2, 1, 1, 5, MotorType::INKEXBOT);
    pack_motor(node_3->medulla_cmd_.motor_3, 1, 2, 6, MotorType::INKEXBOT);
    pack_motor(node_3->medulla_cmd_.motor_4, 1, 3, 7, MotorType::INKEXBOT);
    node_3->medulla_cmd_.motor_5 = 0;
    node_3->medulla_cmd_.motor_6 = 0;
    pack_motor(node_3->medulla_cmd_.motor_7, 4, 0, 20, MotorType::INKEXBOT);
    pack_motor(node_3->medulla_cmd_.motor_8, 4, 1, 21, MotorType::INKEXBOT);
    pack_motor(node_3->medulla_cmd_.motor_9, 4, 2, 22, MotorType::INKEXBOT);

    /*
      第三步 命令下发
    */
    // cmd write
    rt_ethercat_set_command();
    // 这个函数看起来是用于实时以太网通信的关键部分，它负责发送和接收数据，并对通信质量进行监控和错误处理。
    rt_ethercat_run();
  }


  bool E2HW::setupJoints()
  {
    for (const auto &joint : urdfModel_->joints_)
    {
      // int leg_index=0;
      int joint_index=0; 
      int index=0;

      // 根据关节名称确定关节在腿上的索引
      if (joint.first.find("arm_r1_joint") != std::string::npos)
        joint_index = 0;
      else if (joint.first.find("arm_r2_joint") != std::string::npos)
        joint_index = 1;
      else if (joint.first.find("arm_r3_joint") != std::string::npos)
        joint_index = 2;
      else if (joint.first.find("arm_r4_joint") != std::string::npos)
        joint_index = 3;
      else if (joint.first.find("arm_l1_joint") != std::string::npos)
        joint_index = 4;
      else if (joint.first.find("arm_l2_joint") != std::string::npos)
        joint_index = 5;
      else if (joint.first.find("arm_l3_joint") != std::string::npos)
        joint_index = 6;
      else if (joint.first.find("arm_l4_joint") != std::string::npos)
        joint_index = 7;
      else if (joint.first.find("leg_l1_joint") != std::string::npos)
        joint_index = 8;
      else if (joint.first.find("leg_l2_joint") != std::string::npos)
        joint_index = 9;
      else if (joint.first.find("leg_l3_joint") != std::string::npos)
        joint_index = 10;
      else if (joint.first.find("leg_l4_joint") != std::string::npos)
        joint_index = 11;
      else if (joint.first.find("leg_l5_joint") != std::string::npos)
        joint_index = 12;
      else if (joint.first.find("leg_l6_joint") != std::string::npos)
        joint_index = 13;
      else if (joint.first.find("leg_r1_joint") != std::string::npos)
        joint_index = 14;
      else if (joint.first.find("leg_r2_joint") != std::string::npos)
        joint_index = 15;
      else if (joint.first.find("leg_r3_joint") != std::string::npos)
        joint_index = 16;
      else if (joint.first.find("leg_r4_joint") != std::string::npos)
        joint_index = 17;
      else if (joint.first.find("leg_r5_joint") != std::string::npos)
        joint_index = 18;
      else if (joint.first.find("leg_r6_joint") != std::string::npos)
        joint_index = 19;
      else if (joint.first.find("waist_1_joint") != std::string::npos)
        joint_index = 20;
      else if (joint.first.find("waist_2_joint") != std::string::npos)
        joint_index = 21;
      else if (joint.first.find("waist_3_joint") != std::string::npos)
        joint_index = 22;
      else
        continue; 

      // 计算该关节在joint_data_数组中的索引
      index+=joint_index;
      ROS_INFO("joint index = %d", index);

      // 创建JointStateHandle对象并注册到jointStateInterface_
      hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                        &joint_data_[index].tau_);
      jointStateInterface_.registerHandle(state_handle);

      // 创建HybridJointHandle对象并注册到hybridJointInterface_
      hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                            &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                            &joint_data_[index].kd_, &joint_data_[index].ff_));
    } 
    return true;
  }


  bool E2HW::setupImu()
  {
     imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
        "base_imu", "base_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
        imu_data_.linear_acc, imu_data_.linear_acc_cov));
    imu_data_.ori_cov[0] = 0.0012;
    imu_data_.ori_cov[4] = 0.0012;
    imu_data_.ori_cov[8] = 0.0012;

    imu_data_.angular_vel_cov[0] = 0.0004;
    imu_data_.angular_vel_cov[4] = 0.0004;
    imu_data_.angular_vel_cov[8] = 0.0004;

    return true;
  }

  void E2HW::CalibrateImu(const bool is_sim) {

    // get ImuCalibrationParam
    Eigen::Quaternion<double> quat_pre;
    Eigen::Quaternion<double> quat_post;
    if (is_sim) {
        quat_pre.x() = 0;
        quat_pre.y() = 0;
        quat_pre.z() = 0;
        quat_pre.w() = 1;
        quat_post.x() = 0;
        quat_post.y() = 0;
        quat_post.z() = 0;
        quat_post.w() = 1;
    } else {
        quat_pre.x() = 0;
        quat_pre.y() = 0;
        quat_pre.z() = 0;
        quat_pre.w() = 1;
        quat_post.x() = 0;
        quat_post.y() = 0;
        quat_post.z() = 1;
        quat_post.w() = 0;
    }
    Eigen::Matrix3d mat_pre;
    Eigen::Matrix3d mat_post;
    Eigen::Matrix3d mat_output;
    mat_pre = ori_tools_.quatToRotMatrix(quat_pre);
    mat_post = ori_tools_.quatToRotMatrix(quat_post);

    // solve quat AKA orientation
    Eigen::Quaternion<double> quat_input;
    Eigen::Quaternion<double> quat_output;

    quat_input.x() = (double)imu_rc->imu_rc_data_.q1;
    quat_input.y() = (double)imu_rc->imu_rc_data_.q2;
    quat_input.z() = (double)imu_rc->imu_rc_data_.q3;
    quat_input.w() = (double)imu_rc->imu_rc_data_.q0;

    mat_output = mat_pre * ori_tools_.quatToRotMatrix(quat_input) * mat_post;

    quat_output = ori_tools_.rotationMatrixToQuaternion(mat_output);

    imu_data_.ori[0] = quat_output.x();
    imu_data_.ori[1] = quat_output.y();
    imu_data_.ori[2] = quat_output.z();
    imu_data_.ori[3] = quat_output.w();

    // rpy angle
    // robot_state->imu_rpy_angle = ori_tools_.quatToEulerAngle(quat_output);

    // solve gyro AKA angular velocity
    Eigen::Vector3d gyro_input;
    Eigen::Vector3d gyro_output;

    gyro_input << (double)imu_rc->imu_rc_data_.gyr_x,
                  (double)imu_rc->imu_rc_data_.gyr_y,
                  (double)imu_rc->imu_rc_data_.gyr_z;
      
    gyro_output = mat_post * gyro_input;

    imu_data_.angular_vel[0] = gyro_output(0);
    imu_data_.angular_vel[1] = gyro_output(1);
    imu_data_.angular_vel[2] = gyro_output(2);

    // solve acc AKA linear acceleration
    Eigen::Vector3d acc_input;
    Eigen::Vector3d acc_output;

    acc_input << (double)imu_rc->imu_rc_data_.acc_x,
                 (double)imu_rc->imu_rc_data_.acc_y,
                 (double)imu_rc->imu_rc_data_.acc_z;
    if(is_sim) {
        acc_output = mat_post * acc_input;
    } else {
        acc_output = mat_post * acc_input * 9.81;
    }

    imu_data_.linear_acc[0] = acc_output(0);
    imu_data_.linear_acc[1] = acc_output(1);
    imu_data_.linear_acc[2] = acc_output(2);
  }

  bool E2HW::setupParallelSolvers(const PjsolOptionsFromYaml& options)
  {
    std::string error;
    if (!ankle_left.init("left_ankle", options.left_ankle, &error))
    {
      ROS_ERROR("[E2HW] left_ankle init failed: %s", error.c_str());
      return false;
    }
    if (!ankle_right.init("right_ankle", options.right_ankle, &error))
    {
      ROS_ERROR("[E2HW] right_ankle init failed: %s", error.c_str());
      return false;
    }
    if (!waist.init("waist", options.waist, &error))
    {
      ROS_ERROR("[E2HW] waist init failed: %s", error.c_str());
      return false;
    }
    ROS_INFO("[E2HW] pjsol ready: left_ankle=%s right_ankle=%s waist=%s",
             options.left_ankle.type.c_str(), options.right_ankle.type.c_str(),
             options.waist.type.c_str());
    return true;
  }

  void E2HW::parallel_ankle_solve_state(E2MotorData joint_data[])
  {
    const int left_q2_channel = ankle_left.q2Channel();
    const int left_q1_channel = ankle_left.q1Channel();
    const int right_q2_channel = ankle_right.q2Channel();
    const int right_q1_channel = ankle_right.q1Channel();

    if (enable_ankle_log_)
    {
      auto& values = pendingAnkleDebugSample_.values;
      values[0] = joint_data[left_q2_channel].pos_;
      values[1] = joint_data[left_q1_channel].pos_;
      values[2] = joint_data[right_q2_channel].pos_;
      values[3] = joint_data[right_q1_channel].pos_;
      values[4] = joint_data[left_q2_channel].vel_;
      values[5] = joint_data[left_q1_channel].vel_;
      values[6] = joint_data[right_q2_channel].vel_;
      values[7] = joint_data[right_q1_channel].vel_;
      values[8] = joint_data[left_q2_channel].ff_;
      values[9] = joint_data[left_q1_channel].ff_;
      values[10] = joint_data[right_q2_channel].ff_;
      values[11] = joint_data[right_q1_channel].ff_;
    }

    ankle_left.updateState(joint_data);
    ankle_right.updateState(joint_data);

    if (enable_ankle_log_)
    {
      auto& values = pendingAnkleDebugSample_.values;
      values[12] = joint_data[left_q2_channel].pos_;
      values[13] = joint_data[left_q1_channel].pos_;
      values[14] = joint_data[right_q2_channel].pos_;
      values[15] = joint_data[right_q1_channel].pos_;
      values[16] = joint_data[left_q2_channel].vel_;
      values[17] = joint_data[left_q1_channel].vel_;
      values[18] = joint_data[right_q2_channel].vel_;
      values[19] = joint_data[right_q1_channel].vel_;
      values[20] = joint_data[left_q2_channel].ff_;
      values[21] = joint_data[left_q1_channel].ff_;
      values[22] = joint_data[right_q2_channel].ff_;
      values[23] = joint_data[right_q1_channel].ff_;
      ankleDebugSamplePending_ = true;
    }
  }

  void E2HW::parallel_ankle_solve_cmd(E2MotorData joint_data[])
  {
    ankle_left.applyCommand(joint_data);
    ankle_right.applyCommand(joint_data);

    if (enable_ankle_log_ && ankleDebugSamplePending_)
    {
      const int left_q2_channel = ankle_left.q2Channel();
      const int left_q1_channel = ankle_left.q1Channel();
      const int right_q2_channel = ankle_right.q2Channel();
      const int right_q1_channel = ankle_right.q1Channel();
      auto& values = pendingAnkleDebugSample_.values;
      values[24] = joint_data[left_q2_channel].pos_;
      values[25] = joint_data[left_q1_channel].pos_;
      values[26] = joint_data[right_q2_channel].pos_;
      values[27] = joint_data[right_q1_channel].pos_;
      values[28] = joint_data[left_q2_channel].vel_;
      values[29] = joint_data[left_q1_channel].vel_;
      values[30] = joint_data[right_q2_channel].vel_;
      values[31] = joint_data[right_q1_channel].vel_;
      values[32] = joint_data[left_q2_channel].pos_des_;
      values[33] = joint_data[left_q1_channel].pos_des_;
      values[34] = joint_data[right_q2_channel].pos_des_;
      values[35] = joint_data[right_q1_channel].pos_des_;
      values[36] = joint_data[left_q2_channel].vel_des_;
      values[37] = joint_data[left_q1_channel].vel_des_;
      values[38] = joint_data[right_q2_channel].vel_des_;
      values[39] = joint_data[right_q1_channel].vel_des_;
      values[40] = joint_data[left_q2_channel].kp_;
      values[41] = joint_data[left_q1_channel].kp_;
      values[42] = joint_data[right_q2_channel].kp_;
      values[43] = joint_data[right_q1_channel].kp_;
      values[44] = joint_data[left_q2_channel].kd_;
      values[45] = joint_data[left_q1_channel].kd_;
      values[46] = joint_data[right_q2_channel].kd_;
      values[47] = joint_data[right_q1_channel].kd_;
      values[48] = joint_data[left_q2_channel].ff_;
      values[49] = joint_data[left_q1_channel].ff_;
      values[50] = joint_data[right_q2_channel].ff_;
      values[51] = joint_data[right_q1_channel].ff_;
      if (!ankleDebugBuffer_.push(pendingAnkleDebugSample_))
      {
        ankleDebugDropped_.fetch_add(1U, std::memory_order_relaxed);
      }
      ankleDebugSamplePending_ = false;
    }
  }

  void E2HW::parallel_waist_solve_state(E2MotorData joint_data[])
  {
    waist.updateState(joint_data);
  }

  void E2HW::parallel_waist_solve_cmd(E2MotorData joint_data[])
  {
    waist.applyCommand(joint_data);
  }

} // namespace legged
