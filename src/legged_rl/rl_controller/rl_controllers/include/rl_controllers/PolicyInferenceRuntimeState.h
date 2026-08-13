#pragma once

#include "rl_controllers/LatestTripleBuffer.h"
#include "rl_controllers/PolicyRuntimeTypes.h"
#include <legged_common/SpscRingBuffer.h>

#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace legged
{
class PolicyInferenceRuntimeState
{
public:
  void initializeBuffers(size_t maxObservationSize, size_t maxActionSize,
                         size_t maxHistorySize, size_t maxCommandSize);

  ObservationPacket& beginObservation() noexcept;
  void publishObservation() noexcept;
  const ObservationPacket* consumeObservation() noexcept;
  uint64_t observationNotificationGeneration() const noexcept;
  void waitForObservationNotification(uint64_t observedGeneration);
  void waitUntilInferenceActive();
  void notifyInferenceStateChange() noexcept;

  ActionPacket& beginAction() noexcept;
  void publishAction() noexcept;
  const ActionPacket* consumeAction() noexcept;

  TracePacket& beginSynchronousTrace() noexcept;
  void publishSynchronousTrace() noexcept;
  const TracePacket* consumeSynchronousTrace() noexcept;

  std::thread inferenceThread;
  std::thread timingPublisherThread;
  std::thread shutdownWatcherThread;
  std::atomic_bool inferenceRunning{false};
  std::atomic_bool timingPublisherRunning{false};
  std::atomic_bool shutdownWatcherRunning{false};
  std::atomic_bool terminationRequested{false};
  std::atomic<int> setupStatus{0};
  std::mutex shutdownWatcherMutex;
  std::condition_variable shutdownWatcherCv;
  std::atomic_bool controllerActive{false};
  std::atomic<uint64_t> policyEpoch{0};
  std::atomic<uint64_t> lastInferenceUs{0};
  std::atomic<uint64_t> overwrittenObservations{0};
  std::atomic<uint64_t> inferenceFailures{0};

  SpscRingBuffer<InferenceTimingSample, 2048> inferenceTimingBuffer;
  SpscRingBuffer<ActionTimingSample, 2048> actionTimingBuffer;
  std::atomic<uint64_t> inferenceTimingDropped{0};
  std::atomic<uint64_t> actionTimingDropped{0};

  ObservationPacket synchronousObservation;
  ActionPacket synchronousAction;

private:
  LatestTripleBuffer<ObservationPacket> observationBuffer_;
  LatestTripleBuffer<ActionPacket> actionBuffer_;
  LatestTripleBuffer<TracePacket> synchronousTraceBuffer_;
  std::atomic<uint64_t> observationNotificationGeneration_{0};
  alignas(uint32_t) std::atomic<uint32_t> inferenceWakeGeneration_{0};
};
}  // namespace legged
