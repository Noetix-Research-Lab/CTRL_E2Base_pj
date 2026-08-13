#include "rl_controllers/LatestTripleBuffer.h"
#include "rl_controllers/PolicyInferenceRuntimeState.h"

#include <legged_common/SpscRingBuffer.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

namespace legged
{
struct StressPacket
{
  uint64_t sequence{0};
  uint64_t inverse{~uint64_t{0}};
  uint64_t checksum{0};
};

TEST(RealtimeBuffersStressTest, TripleBufferNeverExposesTornPackets)
{
  constexpr uint64_t kIterations = 1000000;
  LatestTripleBuffer<StressPacket> buffer;
  std::atomic_bool producerDone{false};
  std::thread producer([&] {
    for (uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      auto& packet = buffer.producerBuffer();
      packet.sequence = sequence;
      packet.inverse = ~sequence;
      packet.checksum = sequence ^ packet.inverse;
      buffer.publish();
    }
    producerDone.store(true, std::memory_order_release);
  });

  uint64_t lastSequence = 0;
  while (!producerDone.load(std::memory_order_acquire) || lastSequence < kIterations) {
    if (const StressPacket* packet = buffer.consumeLatest()) {
      EXPECT_GT(packet->sequence, lastSequence);
      EXPECT_EQ(packet->inverse, ~packet->sequence);
      EXPECT_EQ(packet->checksum, packet->sequence ^ packet->inverse);
      lastSequence = packet->sequence;
    }
  }
  producer.join();
  EXPECT_EQ(lastSequence, kIterations);
}

TEST(RealtimeBuffersStressTest, RingBufferPreservesEveryPacketInOrder)
{
  constexpr uint64_t kIterations = 500000;
  SpscRingBuffer<StressPacket, 256> buffer;
  std::atomic_bool producerDone{false};
  std::thread producer([&] {
    for (uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      const StressPacket packet{sequence, ~sequence, sequence ^ ~sequence};
      while (!buffer.push(packet)) std::this_thread::yield();
    }
    producerDone.store(true, std::memory_order_release);
  });

  uint64_t expected = 1;
  while (!producerDone.load(std::memory_order_acquire) || expected <= kIterations) {
    StressPacket packet;
    if (!buffer.pop(packet)) {
      std::this_thread::yield();
      continue;
    }
    ASSERT_EQ(packet.sequence, expected++);
    ASSERT_EQ(packet.inverse, ~packet.sequence);
    ASSERT_EQ(packet.checksum, packet.sequence ^ packet.inverse);
  }
  producer.join();
  EXPECT_EQ(expected, kIterations + 1);
}

TEST(RealtimeBuffersStressTest, InferenceRuntimeOwnsMailboxLifecycle)
{
  PolicyInferenceRuntimeState runtime;
  runtime.initializeBuffers(12, 6, 8, 4);

  auto& first = runtime.beginObservation();
  EXPECT_EQ(first.data.size(), 12U);
  EXPECT_EQ(first.history.size(), 8U);
  EXPECT_EQ(first.command.size(), 4U);
  first.sequence = 1;
  runtime.publishObservation();

  auto& latest = runtime.beginObservation();
  latest.sequence = 2;
  runtime.publishObservation();
  EXPECT_EQ(runtime.overwrittenObservations.load(std::memory_order_relaxed), 1U);
  const ObservationPacket* consumedObservation = runtime.consumeObservation();
  ASSERT_NE(consumedObservation, nullptr);
  EXPECT_EQ(consumedObservation->sequence, 2U);
  EXPECT_EQ(runtime.consumeObservation(), nullptr);

  auto& action = runtime.beginAction();
  EXPECT_EQ(action.actions.size(), 6U);
  action.sourceSequence = 2;
  runtime.publishAction();
  const ActionPacket* consumedAction = runtime.consumeAction();
  ASSERT_NE(consumedAction, nullptr);
  EXPECT_EQ(consumedAction->sourceSequence, 2U);

  auto& trace = runtime.beginSynchronousTrace();
  EXPECT_EQ(trace.observation.size(), 12U);
  EXPECT_EQ(trace.action.size(), 6U);
  EXPECT_EQ(trace.history.size(), 8U);
  EXPECT_EQ(trace.command.size(), 4U);
  runtime.publishSynchronousTrace();
  EXPECT_NE(runtime.consumeSynchronousTrace(), nullptr);
}

TEST(RealtimeBuffersStressTest, InferenceMailboxNeverExposesTornPayloads)
{
  constexpr uint64_t kIterations = 500000;
  constexpr size_t kPayloadSize = 128;
  PolicyInferenceRuntimeState runtime;
  runtime.initializeBuffers(kPayloadSize, 24, 64, 32);

  const size_t observationCapacity = runtime.beginObservation().data.capacity();
  std::atomic_bool producerDone{false};
  std::thread producer([&] {
    for (uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      ObservationPacket& packet = runtime.beginObservation();
      packet.sequence = sequence;
      packet.epoch = ~sequence;
      packet.size = kPayloadSize;
      const float marker = static_cast<float>(sequence & 0xffffU);
      std::fill_n(packet.data.begin(), kPayloadSize, marker);
      runtime.publishObservation();
    }
    producerDone.store(true, std::memory_order_release);
  });

  bool valid = true;
  uint64_t lastSequence = 0;
  while (!producerDone.load(std::memory_order_acquire) ||
         lastSequence < kIterations) {
    const ObservationPacket* packet = runtime.consumeObservation();
    if (packet == nullptr) {
      std::this_thread::yield();
      continue;
    }
    const float marker = static_cast<float>(packet->sequence & 0xffffU);
    valid = valid && packet->sequence > lastSequence &&
            packet->epoch == ~packet->sequence &&
            packet->size == kPayloadSize &&
            std::all_of(packet->data.begin(),
                        packet->data.begin() + kPayloadSize,
                        [marker](float value) { return value == marker; });
    lastSequence = packet->sequence;
  }
  producer.join();

  EXPECT_TRUE(valid);
  EXPECT_EQ(lastSequence, kIterations);
  EXPECT_EQ(runtime.beginObservation().data.capacity(), observationCapacity);
}

TEST(RealtimeBuffersStressTest, ObservationNotificationWakesWithoutPolling)
{
  PolicyInferenceRuntimeState runtime;
  runtime.initializeBuffers(12, 6, 8, 4);
  runtime.inferenceRunning.store(true, std::memory_order_release);
  runtime.controllerActive.store(true, std::memory_order_release);
  const uint64_t generation = runtime.observationNotificationGeneration();
  std::atomic_bool waiterStarted{false};
  std::atomic_bool consumed{false};
  std::thread consumer([&] {
    waiterStarted.store(true, std::memory_order_release);
    runtime.waitForObservationNotification(generation);
    const ObservationPacket* packet = runtime.consumeObservation();
    consumed.store(packet != nullptr && packet->sequence == 42,
                   std::memory_order_release);
  });
  while (!waiterStarted.load(std::memory_order_acquire)) std::this_thread::yield();

  auto& packet = runtime.beginObservation();
  packet.sequence = 42;
  const auto publishTime = std::chrono::steady_clock::now();
  runtime.publishObservation();
  consumer.join();
  const auto wakeLatency = std::chrono::steady_clock::now() - publishTime;
  EXPECT_TRUE(consumed.load(std::memory_order_acquire));
  EXPECT_LT(wakeLatency, std::chrono::milliseconds(100));
}

TEST(RealtimeBuffersStressTest, ObservationWakeIsNotLostAtWaitBoundary)
{
  constexpr uint64_t kIterations = 20000;
  PolicyInferenceRuntimeState runtime;
  runtime.initializeBuffers(12, 6, 8, 4);
  runtime.inferenceRunning.store(true, std::memory_order_release);
  runtime.controllerActive.store(true, std::memory_order_release);

  std::atomic<uint64_t> waiterReady{0};
  std::atomic<uint64_t> completed{0};
  std::atomic_bool valid{true};
  std::thread consumer([&] {
    for (uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      const uint64_t generation = runtime.observationNotificationGeneration();
      waiterReady.store(sequence, std::memory_order_release);
      runtime.waitForObservationNotification(generation);
      const ObservationPacket* packet = runtime.consumeObservation();
      if (packet == nullptr || packet->sequence != sequence) {
        valid.store(false, std::memory_order_release);
        return;
      }
      completed.store(sequence, std::memory_order_release);
    }
  });

  for (uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
    while (waiterReady.load(std::memory_order_acquire) != sequence)
      std::this_thread::yield();
    ObservationPacket& packet = runtime.beginObservation();
    packet.sequence = sequence;
    runtime.publishObservation();
    while (completed.load(std::memory_order_acquire) != sequence)
      std::this_thread::yield();
  }
  consumer.join();
  EXPECT_TRUE(valid.load(std::memory_order_acquire));
}

TEST(RealtimeBuffersStressTest, StateChangeWakesIdleInferenceThread)
{
  PolicyInferenceRuntimeState runtime;
  runtime.initializeBuffers(12, 6, 8, 4);
  runtime.inferenceRunning.store(true, std::memory_order_release);
  runtime.controllerActive.store(false, std::memory_order_release);
  std::atomic_bool activated{false};
  std::thread consumer([&] {
    runtime.waitUntilInferenceActive();
    activated.store(runtime.controllerActive.load(std::memory_order_acquire),
                    std::memory_order_release);
  });
  runtime.controllerActive.store(true, std::memory_order_release);
  runtime.notifyInferenceStateChange();
  consumer.join();
  EXPECT_TRUE(activated.load(std::memory_order_acquire));

  runtime.controllerActive.store(true, std::memory_order_release);
  const uint64_t generation = runtime.observationNotificationGeneration();
  std::atomic_bool stopped{false};
  std::thread stoppingConsumer([&] {
    runtime.waitForObservationNotification(generation);
    stopped.store(!runtime.inferenceRunning.load(std::memory_order_acquire),
                  std::memory_order_release);
  });
  runtime.inferenceRunning.store(false, std::memory_order_release);
  runtime.notifyInferenceStateChange();
  stoppingConsumer.join();
  EXPECT_TRUE(stopped.load(std::memory_order_acquire));
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
