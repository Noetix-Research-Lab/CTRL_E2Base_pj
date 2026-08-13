#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

namespace legged
{
// Allocation-free absolute-time scheduler used by every policy mode. A late
// control cycle skips stale ticks instead of accumulating phase error.
class PolicyPlaybackClock
{
public:
  void reset(int64_t nowNs) noexcept
  {
    startNs_ = nowNs;
    nextPolicyNs_ = nowNs;
    lastPolicyTick_ = 0;
    hasProcessedTick_ = false;
  }

  bool take(int64_t policyPeriodNs, int64_t nowNs,
            uint64_t& policyTick) noexcept
  {
    if (startNs_ <= 0 || policyPeriodNs <= 0 || nowNs < nextPolicyNs_)
      return false;

    const int64_t elapsedNs = std::max<int64_t>(0, nowNs - startNs_);
    policyTick = static_cast<uint64_t>(elapsedNs / policyPeriodNs);
    if (hasProcessedTick_ && policyTick == lastPolicyTick_) return false;

    lastPolicyTick_ = policyTick;
    hasProcessedTick_ = true;
    constexpr int64_t kMaxNs = std::numeric_limits<int64_t>::max();
    const uint64_t maxNextTick = static_cast<uint64_t>(
        (kMaxNs - startNs_) / policyPeriodNs);
    nextPolicyNs_ = policyTick >= maxNextTick
        ? kMaxNs
        : startNs_ + static_cast<int64_t>(policyTick + 1) * policyPeriodNs;
    return true;
  }

  int64_t startNs() const noexcept { return startNs_; }
  int64_t nextPolicyNs() const noexcept { return nextPolicyNs_; }

private:
  int64_t startNs_{0};
  int64_t nextPolicyNs_{0};
  uint64_t lastPolicyTick_{0};
  bool hasProcessedTick_{false};
};
}  // namespace legged
