#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace legged
{

/**
 * Single-producer/single-consumer latest-value mailbox.
 *
 * The producer owns backIndex_, the consumer owns frontIndex_, and middleState_
 * atomically transfers ownership of the third slot. Publishing again before a
 * consume replaces the pending value, so latency cannot grow through queuing.
 * Slots must be fully initialized before producer/consumer access starts.
 */
template <typename T>
class LatestTripleBuffer
{
public:
  LatestTripleBuffer() = default;
  LatestTripleBuffer(const LatestTripleBuffer&) = delete;
  LatestTripleBuffer& operator=(const LatestTripleBuffer&) = delete;

  template <typename Initializer>
  void initialize(Initializer&& initializer)
  {
    for (auto& slot : slots_) {
      initializer(slot);
    }
  }

  T& producerBuffer() noexcept
  {
    return slots_[backIndex_];
  }

  bool publish() noexcept
  {
    const uint32_t previous = middleState_.exchange(backIndex_ | kDirtyBit,
                                                     std::memory_order_acq_rel);
    backIndex_ = previous & kIndexMask;
    return (previous & kDirtyBit) != 0U;
  }

  const T* consumeLatest() noexcept
  {
    uint32_t state = middleState_.load(std::memory_order_acquire);
    while ((state & kDirtyBit) != 0U) {
      const uint32_t desired = frontIndex_;
      if (middleState_.compare_exchange_weak(state, desired,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
        frontIndex_ = state & kIndexMask;
        return &slots_[frontIndex_];
      }
    }
    return nullptr;
  }

private:
  static constexpr uint32_t kDirtyBit = uint32_t{1} << 31U;
  static constexpr uint32_t kIndexMask = 0x3U;
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "LatestTripleBuffer requires lock-free 32-bit atomics");

  std::array<T, 3> slots_{};
  std::atomic<uint32_t> middleState_{1U};
  uint32_t backIndex_{2U};
  uint32_t frontIndex_{0U};
};

}  // namespace legged
