#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace legged
{

/** Fixed-capacity, allocation-free single-producer/single-consumer queue. */
template <typename T, size_t Capacity>
class SpscRingBuffer
{
  static_assert(Capacity >= 2, "SpscRingBuffer needs at least two slots");

public:
  bool push(const T& value) noexcept
  {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = increment(head);
    if (next == tail_.load(std::memory_order_acquire)) return false;
    slots_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T& value) noexcept
  {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    value = slots_[tail];
    tail_.store(increment(tail), std::memory_order_release);
    return true;
  }

private:
  static constexpr size_t increment(size_t index) noexcept
  {
    return (index + 1U) % Capacity;
  }

  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
};

}  // namespace legged
