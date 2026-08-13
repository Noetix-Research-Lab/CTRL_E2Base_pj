#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace legged
{

enum class CanChannelId : std::uint8_t
{
  Node1Can1 = 0,
  Node1Can2,
  Node1Can3,
  Node2Can1,
  Node2Can2,
  Node2Can3,
  Node3Can1,
  Node3Can2,
  Node3Can3,
  Count
};

enum class CanState : std::uint8_t
{
  Normal = 0,
  Fault
};

// Candidate joints are derived from the fixed Medulla motor-slot mapping in
// E2HW. A channel-level error counter cannot identify one exact failed motor.
// Use '|' as the separator so this string remains one field in the CAN CSV.
inline const char* CanChannelCandidateMotors(CanChannelId channel) noexcept
{
  switch (channel)
  {
    case CanChannelId::Node1Can1:
      return "leg_l1_joint|leg_l2_joint|leg_l3_joint";
    case CanChannelId::Node1Can2:
      return "leg_l4_joint|leg_l5_joint|leg_l6_joint";
    case CanChannelId::Node1Can3:
      return "arm_r1_joint|arm_r2_joint|arm_r3_joint";
    case CanChannelId::Node2Can1:
      return "leg_r1_joint|leg_r2_joint|leg_r3_joint";
    case CanChannelId::Node2Can2:
      return "leg_r4_joint|leg_r5_joint|leg_r6_joint";
    case CanChannelId::Node2Can3:
      return "arm_r4_joint";
    case CanChannelId::Node3Can1:
      return "arm_l1_joint|arm_l2_joint|arm_l3_joint";
    case CanChannelId::Node3Can2:
      return "arm_l4_joint";
    case CanChannelId::Node3Can3:
      return "waist_1_joint|waist_2_joint|waist_3_joint";
    case CanChannelId::Count:
      return "unknown";
  }
  return "unknown";
}

struct CanEvent
{
  std::uint32_t value{0};
  std::uint32_t delta{0};
  std::uint32_t streak{0};
  CanChannelId channel{CanChannelId::Node1Can1};
  CanState state{CanState::Normal};
  // Filled by the RT hardware loop at the exact cycle in which the detector
  // emits the transition. The detector itself remains independent of ROS time.
  std::uint32_t stampSec{0};
  std::uint32_t stampNsec{0};
  std::uint64_t cycleSequence{0};
};

/**
 * Process-lifetime fail-closed latch for Medulla CAN channel faults.
 *
 * A recovery event clears only the live-fault mask. The latched mask is
 * intentionally monotonic so a transient motor-bus failure cannot resume
 * control automatically. Clearing it requires restarting the hardware node.
 */
class CanFaultLatch
{
public:
  void update(const CanEvent& event) noexcept
  {
    const auto index = static_cast<std::uint8_t>(event.channel);
    if (index >= static_cast<std::uint8_t>(CanChannelId::Count))
    {
      return;
    }

    const std::uint16_t bit = static_cast<std::uint16_t>(
        std::uint16_t{1} << index);
    if (event.state == CanState::Fault)
    {
      activeMask_.fetch_or(bit, std::memory_order_release);
      latchedMask_.fetch_or(bit, std::memory_order_release);
    }
    else
    {
      activeMask_.fetch_and(static_cast<std::uint16_t>(~bit),
                            std::memory_order_release);
    }
  }

  bool faulted() const noexcept
  {
    return latchedMask() != 0U;
  }

  std::uint16_t activeMask() const noexcept
  {
    return activeMask_.load(std::memory_order_acquire);
  }

  std::uint16_t latchedMask() const noexcept
  {
    return latchedMask_.load(std::memory_order_acquire);
  }

private:
  std::atomic<std::uint16_t> activeMask_{0U};
  std::atomic<std::uint16_t> latchedMask_{0U};
};

/**
 * Allocation-free detector for the cumulative CAN error counters in Medulla PDOs.
 *
 * update() is intended for the RT thread. The supplied sink must also be
 * non-blocking and allocation-free (the hardware implementation uses an SPSC
 * ring buffer). Invalid/stale EtherCAT samples must not be passed to update().
 */
class CanErrorDetector
{
public:
  static constexpr std::size_t kChannelCount =
      static_cast<std::size_t>(CanChannelId::Count);
  using CounterArray = std::array<std::uint32_t, kChannelCount>;

  CanErrorDetector() noexcept = default;

  void configure(std::uint32_t deltaThreshold,
                 std::uint32_t faultSamples,
                 std::uint32_t recoverySamples,
                 std::uint16_t enabledMask) noexcept
  {
    deltaThreshold_ = deltaThreshold;
    faultSamples_ = faultSamples == 0U ? 1U : faultSamples;
    recoverySamples_ = recoverySamples == 0U ? 1U : recoverySamples;
    enabledMask_ = static_cast<std::uint16_t>(enabledMask & kAllChannelsMask);
    reset();
  }

  void reset() noexcept
  {
    monitors_ = {};
  }

  template <typename EventSink>
  void update(const CounterArray& counters, EventSink&& sink) noexcept
  {
    for (std::size_t index = 0; index < kChannelCount; ++index)
    {
      if ((enabledMask_ & (std::uint16_t{1} << index)) == 0U)
      {
        continue;
      }
      updateChannel(static_cast<CanChannelId>(index), counters[index],
                    monitors_[index], sink);
    }
  }

  CanState state(CanChannelId channel) const noexcept
  {
    return monitors_[static_cast<std::size_t>(channel)].state;
  }

  static constexpr std::uint16_t kAllChannelsMask =
      static_cast<std::uint16_t>((std::uint16_t{1} << kChannelCount) - 1U);

private:
  struct ChannelMonitor
  {
    std::uint32_t lastValue{0};
    std::uint32_t badStreak{0};
    std::uint32_t goodStreak{0};
    CanState state{CanState::Normal};
    bool initialized{false};
  };

  static void incrementSaturated(std::uint32_t& value,
                                 std::uint32_t limit) noexcept
  {
    if (value < limit)
    {
      ++value;
    }
  }

  template <typename EventSink>
  void updateChannel(CanChannelId channel, std::uint32_t current,
                     ChannelMonitor& monitor, EventSink&& sink) noexcept
  {
    if (!monitor.initialized)
    {
      monitor.lastValue = current;
      monitor.initialized = true;
      return;
    }

    // A lower value means either a board reset or the very rare uint32 wrap.
    // Treat that sample as zero delta to avoid a false disconnect event.
    const std::uint32_t delta =
        current >= monitor.lastValue ? current - monitor.lastValue : 0U;
    monitor.lastValue = current;

    if (delta > deltaThreshold_)
    {
      monitor.goodStreak = 0U;
      incrementSaturated(monitor.badStreak, faultSamples_);
      if (monitor.badStreak >= faultSamples_ && monitor.state != CanState::Fault)
      {
        monitor.state = CanState::Fault;
        sink(CanEvent{current, delta, monitor.badStreak, channel, CanState::Fault});
      }
      return;
    }

    monitor.badStreak = 0U;
    incrementSaturated(monitor.goodStreak, recoverySamples_);
    if (monitor.goodStreak >= recoverySamples_ && monitor.state == CanState::Fault)
    {
      monitor.state = CanState::Normal;
      sink(CanEvent{current, delta, monitor.goodStreak, channel, CanState::Normal});
    }
  }

  std::array<ChannelMonitor, kChannelCount> monitors_{};
  std::uint32_t deltaThreshold_{200U};
  std::uint32_t faultSamples_{50U};
  std::uint32_t recoverySamples_{50U};
  std::uint16_t enabledMask_{kAllChannelsMask};
};

}  // namespace legged
