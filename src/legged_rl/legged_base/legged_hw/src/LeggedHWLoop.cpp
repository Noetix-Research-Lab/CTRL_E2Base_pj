/********************************************************************************
Modified Copyright (c) 2023-2024, Noetix Robotics.Co.Ltd. All rights reserved.

For further information, contact: huaxinghuang@noetixrobotics.com or visit our website
at www.noetixrobotics.com.
********************************************************************************/

#include "legged_hw/LeggedHWLoop.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <utility>

namespace
{
constexpr std::size_t kRtStackPrefaultBytes = 256U * 1024U;

#if defined(__GNUC__)
__attribute__((noinline))
#endif
void prefaultCurrentThreadStack() noexcept
{
  long pageSize = sysconf(_SC_PAGESIZE);
  if (pageSize <= 0)
  {
    pageSize = 4096;
  }

  // Volatile writes prevent the compiler from removing the page touches. This
  // storage is intentionally on the current thread's stack, not on the heap.
  volatile std::uint8_t stackPages[kRtStackPrefaultBytes];
  const std::size_t stride = static_cast<std::size_t>(pageSize);
  for (std::size_t offset = 0; offset < kRtStackPrefaultBytes; offset += stride)
  {
    stackPages[offset] = 0U;
  }
  // If the array begins at a non-page-aligned address, the last byte can lie on
  // one additional page that is not covered by the fixed-stride loop.
  stackPages[kRtStackPrefaultBytes - 1U] = 0U;
}
}  // namespace

namespace legged
{
LeggedHWLoop::LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface)
  : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true)
{
  
    // controllerManager_.reset();
  
  controllerManager_ .reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));
  int error = 0;
  int threadPriority = 0;
  int cpuAffinity = -1;
  ros::NodeHandle nhP("~");
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  nhP.param("cpu_affinity", cpuAffinity, -1);
  nhP.param("require_realtime", requireRealtime_, false);
  nhP.param("enable_timing_metrics", enableTimingMetrics_, false);
  if (error > 0)
  {
    std::string error_message =
        "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }
  if (!std::isfinite(loopHz_) || loopHz_ <= 0.0 ||
      !std::isfinite(cycleTimeErrorThreshold_) || cycleTimeErrorThreshold_ < 0.0)
  {
    throw std::runtime_error("loop_frequency must be positive and cycle_time_error_threshold non-negative");
  }
  loopPeriod_ = std::chrono::duration_cast<Clock::duration>(Duration(1.0 / loopHz_));
  if (loopPeriod_ <= Clock::duration::zero())
  {
    throw std::runtime_error("loop_frequency is too high for steady_clock resolution");
  }

  if (enableTimingMetrics_)
  {
    timingPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "/data_analysis/control_loop_timing", 10);
  }

  loopThread_ = std::thread([this]() {
    pthread_setname_np(pthread_self(), "legged_hw_rt");
    // Run once on the RT thread's own stack before it enters the 500 Hz loop.
    // mlockall(MCL_CURRENT | MCL_FUTURE) is set by the hardware-node main.
    prefaultCurrentThreadStack();
    {
      std::unique_lock<std::mutex> lock(startMutex_);
      startCv_.wait(lock, [this]() { return loopThreadConfigured_ || !loopRunning_.load(); });
    }

    while (loopRunning_.load())
    {
      update();
    }
  });

  bool realtimeSetupFailed = false;
  std::string realtimeSetupError;
  if (cpuAffinity >= 0)
  {
    const long nCpus = sysconf(_SC_NPROCESSORS_ONLN);
    if (nCpus > 0 && cpuAffinity < static_cast<int>(nCpus))
    {
      cpu_set_t cpuSet;
      CPU_ZERO(&cpuSet);
      CPU_SET(cpuAffinity, &cpuSet);
      const int affinityRet = pthread_setaffinity_np(loopThread_.native_handle(), sizeof(cpu_set_t), &cpuSet);
      if (affinityRet != 0)
      {
        ROS_WARN("Failed to pin RT control loop thread to core %d: %s",
                 cpuAffinity, std::strerror(affinityRet));
        realtimeSetupFailed = true;
        realtimeSetupError = "failed to pin RT control thread";
      }
      else
      {
        ROS_INFO("RT control loop thread pinned to core %d.", cpuAffinity);
      }
    }
    else
    {
      ROS_WARN("Invalid cpu_affinity=%d for %ld online CPUs; RT control loop thread is not pinned.",
               cpuAffinity, nCpus);
      realtimeSetupFailed = true;
      realtimeSetupError = "invalid RT control-loop CPU affinity";
    }
  }
  else if (requireRealtime_)
  {
    ROS_ERROR("require_realtime is enabled but cpu_affinity is disabled");
    realtimeSetupFailed = true;
    realtimeSetupError = "RT control-loop CPU affinity is required";
  }

  sched_param sched{};
  sched.sched_priority = threadPriority;
  const int schedRet = pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched);
  if (schedRet != 0)
  {
    ROS_WARN(
        "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        "are not set properly.): %s\n",
        std::strerror(schedRet));
    realtimeSetupFailed = true;
    if (realtimeSetupError.empty()) realtimeSetupError = "failed to set RT control-loop SCHED_FIFO priority";
  }

  if (requireRealtime_ && realtimeSetupFailed)
  {
    {
      // The startup waiter evaluates its predicate while holding
      // startMutex_. Change the predicate under the same mutex so the notify
      // cannot fall between the predicate check and wait's atomic
      // unlock-and-block step.
      std::lock_guard<std::mutex> lock(startMutex_);
      loopRunning_.store(false, std::memory_order_release);
    }
    startCv_.notify_one();
    if (loopThread_.joinable()) loopThread_.join();
    throw std::runtime_error("require_realtime: " + realtimeSetupError);
  }

  {
    std::lock_guard<std::mutex> lock(startMutex_);
    lastTime_ = Clock::now();
    nextDeadline_ = lastTime_ + loopPeriod_;
    loopThreadConfigured_ = true;
  }
  startCv_.notify_one();
  // This worker also owns overrun logging, so it runs even when metric
  // publication is disabled. The RT loop never calls ROS logging APIs.
  timingPublisherRunning_.store(true, std::memory_order_release);
  try
  {
    timingPublisherThread_ = std::thread(&LeggedHWLoop::timingPublisherLoop, this);
  }
  catch (...)
  {
    timingPublisherRunning_.store(false, std::memory_order_release);
    loopRunning_.store(false, std::memory_order_release);
    if (loopThread_.joinable())
    {
      loopThread_.join();
    }
    throw;
  }
}

void LeggedHWLoop::update()
{
  const auto currentTime = Clock::now();

  // nextDeadline_ is an absolute steady-clock grid. If the loop is more than
  // one complete period late, discard expired slots instead of executing a
  // burst of catch-up cycles.
  uint64_t expiredSlots = 0;
  if (currentTime >= nextDeadline_)
  {
    const auto periodsLate = (currentTime - nextDeadline_) / loopPeriod_ + 1;
    expiredSlots = static_cast<uint64_t>(periodsLate);
    nextDeadline_ += loopPeriod_ * periodsLate;
  }
  const Clock::time_point cycleDeadline = nextDeadline_;
  const Duration desiredDuration = std::chrono::duration_cast<Duration>(loopPeriod_);

  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  const bool thresholdExceeded = cycle_time_error > cycleTimeErrorThreshold_;
  // ROS time is for external timestamps only. Sample it once so read/update/
  // write observe one consistent timestamp even if wall/sim time changes.
  const ros::Time rosTime = ros::Time::now();

  // ROS_INFO("READ");
  hardwareInterface_->read(rosTime, elapsedTime_);
  // ROS_INFO("UPDATE");
  controllerManager_->update(rosTime, elapsedTime_);
  // ROS_INFO("WRITE");
  hardwareInterface_->write(rosTime, elapsedTime_);
  // ROS_INFO("WRITE_OK");

  const auto workEnd = Clock::now();
  const bool workDeadlineMissed = workEnd > cycleDeadline;
  const bool deadlineMissed = expiredSlots > 0 || workDeadlineMissed;
  if (enableTimingMetrics_ || thresholdExceeded || deadlineMissed)
  {
    const double desiredUs = desiredDuration.count() * 1e6;
    const double periodUs = elapsedTime_.toSec() * 1e6;
    const double workUs = std::chrono::duration<double, std::micro>(workEnd - currentTime).count();
    ControlTimingSample sample;
    sample.sequence = ++timingSequence_;
    sample.startNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        currentTime.time_since_epoch()).count();
    sample.periodUs = periodUs;
    sample.workUs = workUs;
    sample.sleepBudgetUs = desiredUs - workUs;
    sample.periodErrorUs = periodUs - desiredUs;
    sample.deadlineMissed = deadlineMissed ? 1.0 : 0.0;
    sample.thresholdExceeded = thresholdExceeded ? 1.0 : 0.0;
    if (!timingBuffer_.push(sample))
    {
      timingDropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  nextDeadline_ = cycleDeadline + loopPeriod_;
  if (!workDeadlineMissed)
  {
    std::this_thread::sleep_until(cycleDeadline);
  }
}

void LeggedHWLoop::timingPublisherLoop()
{
  pthread_setname_np(pthread_self(), "hw_timing_pub");
  constexpr size_t kColumns = 9;
  constexpr size_t kMaxBatch = 512;
  while (timingPublisherRunning_.load(std::memory_order_acquire))
  {
    const bool publish = enableTimingMetrics_ && timingPublisher_.getNumSubscribers() > 0;
    std_msgs::Float64MultiArray message;
    if (publish)
    {
      message.layout.dim.resize(2);
      message.layout.dim[0].label = "samples";
      message.layout.dim[1].label =
          "sequence,start_ns,period_us,work_us,sleep_budget_us,period_error_us,deadline_missed,threshold_exceeded,dropped_total";
      message.layout.dim[1].size = kColumns;
      message.layout.dim[1].stride = kColumns;
      message.data.reserve(kColumns * 32);
    }

    const double dropped = static_cast<double>(timingDropped_.load(std::memory_order_relaxed));
    ControlTimingSample sample;
    size_t rows = 0;
    while (rows < kMaxBatch && timingBuffer_.pop(sample))
    {
      if (sample.thresholdExceeded > 0.5 || sample.deadlineMissed > 0.5)
      {
        ROS_WARN_THROTTLE(1.0,
                          "Control timing violation: period=%.6fs, error=%.6fs, threshold=%.6fs, deadline_missed=%d",
                          sample.periodUs / 1e6,
                          sample.periodErrorUs / 1e6,
                          cycleTimeErrorThreshold_,
                          sample.deadlineMissed > 0.5 ? 1 : 0);
      }
      if (publish)
      {
        message.data.push_back(static_cast<double>(sample.sequence));
        message.data.push_back(static_cast<double>(sample.startNs));
        message.data.push_back(sample.periodUs);
        message.data.push_back(sample.workUs);
        message.data.push_back(sample.sleepBudgetUs);
        message.data.push_back(sample.periodErrorUs);
        message.data.push_back(sample.deadlineMissed);
        message.data.push_back(sample.thresholdExceeded);
        message.data.push_back(dropped);
      }
      ++rows;
    }
    if (publish && rows > 0)
    {
      message.layout.dim[0].size = rows;
      message.layout.dim[0].stride = rows * kColumns;
      timingPublisher_.publish(message);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

LeggedHWLoop::~LeggedHWLoop()
{
  {
    std::lock_guard<std::mutex> lock(startMutex_);
    loopRunning_.store(false, std::memory_order_release);
    loopThreadConfigured_ = true;
  }
  startCv_.notify_one();
  if (loopThread_.joinable())
  {
    loopThread_.join();
  }
  timingPublisherRunning_.store(false, std::memory_order_release);
  if (timingPublisherThread_.joinable())
  {
    timingPublisherThread_.join();
  }
}

}  // namespace legged
