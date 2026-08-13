#include "rl_controllers/PolicyInferenceRuntimeState.h"

#include <climits>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace
{
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "Inference wake generation requires lock-free 32-bit atomics");
static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
              "Linux futex requires a 32-bit atomic storage word");

void waitForGenerationChange(std::atomic<uint32_t>& generation,
                             uint32_t observedGeneration) noexcept
{
  // FUTEX_WAIT performs the value check and sleep transition atomically in
  // the kernel. EAGAIN means the generation changed before the thread slept;
  // EINTR and spurious wakeups are handled by the caller's predicate loop.
  syscall(SYS_futex, reinterpret_cast<uint32_t*>(&generation),
          FUTEX_WAIT_PRIVATE, observedGeneration, nullptr, nullptr, 0);
}

void advanceGenerationAndWake(std::atomic<uint32_t>& generation,
                              int waiterCount) noexcept
{
  generation.fetch_add(1, std::memory_order_release);
  syscall(SYS_futex, reinterpret_cast<uint32_t*>(&generation),
          FUTEX_WAKE_PRIVATE, waiterCount, nullptr, nullptr, 0);
}
}  // namespace

namespace legged
{
void PolicyInferenceRuntimeState::initializeBuffers(
    size_t maxObservationSize, size_t maxActionSize,
    size_t maxHistorySize, size_t maxCommandSize)
{
  observationNotificationGeneration_.store(0, std::memory_order_relaxed);
  inferenceWakeGeneration_.store(0, std::memory_order_relaxed);
  observationBuffer_.initialize(
      [=](ObservationPacket& packet) {
        packet.data.resize(maxObservationSize, 0.0F);
        packet.history.resize(maxHistorySize, 0.0F);
        packet.command.resize(maxCommandSize, 0.0F);
      });
  actionBuffer_.initialize(
      [=](ActionPacket& packet) {
        packet.actions.resize(maxActionSize, 0.0F);
      });
  synchronousTraceBuffer_.initialize(
      [=](TracePacket& packet) {
        packet.observation.resize(maxObservationSize, 0.0F);
        packet.action.resize(maxActionSize, 0.0F);
        packet.history.resize(maxHistorySize, 0.0F);
        packet.command.resize(maxCommandSize, 0.0F);
      });

  synchronousObservation.data.resize(maxObservationSize, 0.0F);
  synchronousObservation.history.resize(maxHistorySize, 0.0F);
  synchronousObservation.command.resize(maxCommandSize, 0.0F);
  synchronousAction.actions.resize(maxActionSize, 0.0F);
}

ObservationPacket& PolicyInferenceRuntimeState::beginObservation() noexcept
{
  return observationBuffer_.producerBuffer();
}

void PolicyInferenceRuntimeState::publishObservation() noexcept
{
  if (observationBuffer_.publish()) {
    overwrittenObservations.fetch_add(1, std::memory_order_relaxed);
  }
  observationNotificationGeneration_.fetch_add(1, std::memory_order_release);
  advanceGenerationAndWake(inferenceWakeGeneration_, 1);
}

const ObservationPacket* PolicyInferenceRuntimeState::consumeObservation() noexcept
{
  return observationBuffer_.consumeLatest();
}

uint64_t PolicyInferenceRuntimeState::observationNotificationGeneration() const noexcept
{
  return observationNotificationGeneration_.load(std::memory_order_acquire);
}

void PolicyInferenceRuntimeState::waitForObservationNotification(
    uint64_t observedGeneration)
{
  while (observationNotificationGeneration_.load(std::memory_order_acquire) ==
             observedGeneration &&
         inferenceRunning.load(std::memory_order_acquire) &&
         controllerActive.load(std::memory_order_acquire)) {
    const uint32_t wakeGeneration =
        inferenceWakeGeneration_.load(std::memory_order_acquire);
    if (observationNotificationGeneration_.load(std::memory_order_acquire) !=
            observedGeneration ||
        !inferenceRunning.load(std::memory_order_acquire) ||
        !controllerActive.load(std::memory_order_acquire)) {
      break;
    }
    waitForGenerationChange(inferenceWakeGeneration_, wakeGeneration);
  }
}

void PolicyInferenceRuntimeState::waitUntilInferenceActive()
{
  while (!controllerActive.load(std::memory_order_acquire) &&
         inferenceRunning.load(std::memory_order_acquire)) {
    const uint32_t wakeGeneration =
        inferenceWakeGeneration_.load(std::memory_order_acquire);
    // Recheck after taking the generation snapshot. If a state transition
    // raced with the snapshot, its generation increment is either already
    // visible or will make FUTEX_WAIT return instead of losing the wakeup.
    if (controllerActive.load(std::memory_order_acquire) ||
        !inferenceRunning.load(std::memory_order_acquire)) {
      break;
    }
    waitForGenerationChange(inferenceWakeGeneration_, wakeGeneration);
  }
}

void PolicyInferenceRuntimeState::notifyInferenceStateChange() noexcept
{
  // Advancing the futex generation makes activation and shutdown persistent
  // events, so a wake that happens immediately before the waiter blocks
  // cannot be lost.
  advanceGenerationAndWake(inferenceWakeGeneration_, INT_MAX);
}

ActionPacket& PolicyInferenceRuntimeState::beginAction() noexcept
{
  return actionBuffer_.producerBuffer();
}

void PolicyInferenceRuntimeState::publishAction() noexcept
{
  actionBuffer_.publish();
}

const ActionPacket* PolicyInferenceRuntimeState::consumeAction() noexcept
{
  return actionBuffer_.consumeLatest();
}

TracePacket& PolicyInferenceRuntimeState::beginSynchronousTrace() noexcept
{
  return synchronousTraceBuffer_.producerBuffer();
}

void PolicyInferenceRuntimeState::publishSynchronousTrace() noexcept
{
  synchronousTraceBuffer_.publish();
}

const TracePacket* PolicyInferenceRuntimeState::consumeSynchronousTrace() noexcept
{
  return synchronousTraceBuffer_.consumeLatest();
}
}  // namespace legged
