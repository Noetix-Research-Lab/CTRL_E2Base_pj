/********************************************************************************
Modified Copyright (c) 2023-2024, Noetix Robotics.Co.Ltd. All rights reserved.

For further information, contact: huaxinghuang@noetixrobotics.com or visit our website
at www.noetixrobotics.com.
********************************************************************************/

#pragma once

#include "legged_hw/LeggedHW.h"
#include <legged_common/SpscRingBuffer.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace legged
{
class LeggedHWLoop
{  // NOLINT(cppcoreguidelines-special-member-functions)
  // Control-period measurement and sleep deadlines must never move backwards
  // when wall time is adjusted by NTP/PTP or an operator.
  using Clock = std::chrono::steady_clock;
  static_assert(Clock::is_steady, "LeggedHWLoop requires a monotonic clock");
  using Duration = std::chrono::duration<double>;

public:
  /** \brief Create controller manager. Load loop frequency. Start control loop which call @ref
   * legged::RmRobotHWLoop::update() in a frequency.
   *
   * @param nh Node-handle of a ROS node.
   * @param hardware_interface A pointer which point to hardware_interface.
   */
  LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface);

  ~LeggedHWLoop();

  /** \brief Timed method that reads current hardware's state, runs the controller code once and sends the new commands
   * to the hardware.
   *
   * Timed method that reads current hardware's state, runs the controller code once and sends the new commands to the
   * hardware.
   *
   */
  void update();

private:
  struct ControlTimingSample
  {
    uint64_t sequence{0};
    int64_t startNs{0};
    double periodUs{0.0};
    double workUs{0.0};
    double sleepBudgetUs{0.0};
    double periodErrorUs{0.0};
    double deadlineMissed{0.0};
    double thresholdExceeded{0.0};
  };

  void timingPublisherLoop();

  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Timing
  double cycleTimeErrorThreshold_{}, loopHz_{};
  std::thread loopThread_;
  std::atomic_bool loopRunning_{};
  std::mutex startMutex_;
  std::condition_variable startCv_;
  bool loopThreadConfigured_{false};
  ros::Duration elapsedTime_;
  Clock::time_point lastTime_;
  Clock::time_point nextDeadline_;
  Clock::duration loopPeriod_{};
  bool requireRealtime_{false};
  bool enableTimingMetrics_{false};
  uint64_t timingSequence_{0};
  SpscRingBuffer<ControlTimingSample, 4096> timingBuffer_;
  std::atomic<uint64_t> timingDropped_{0};
  ros::Publisher timingPublisher_;
  std::atomic_bool timingPublisherRunning_{false};
  std::thread timingPublisherThread_;

  /** ROS Controller Manager and Runner

      This class advertises a ROS interface for loading, unloading, starting, and
      stopping ros_control-based controllers. It also serializes execution of all
      running controllers in \ref update.
  **/
  std::shared_ptr<controller_manager::ControllerManager>  controllerManager_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<LeggedHW> hardwareInterface_;
};

}  // namespace legged
