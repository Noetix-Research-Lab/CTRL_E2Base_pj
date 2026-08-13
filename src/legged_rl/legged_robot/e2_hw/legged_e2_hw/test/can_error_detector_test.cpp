#include "CanErrorDetector.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace legged
{
namespace
{

TEST(CanErrorDetectorTest, FaultsAndRecoversAtConfiguredStreaks)
{
  CanErrorDetector detector;
  detector.configure(200U, 3U, 2U, CanErrorDetector::kAllChannelsMask);
  CanErrorDetector::CounterArray counters{};
  std::vector<CanEvent> events;
  const auto sink = [&events](const CanEvent& event) { events.push_back(event); };

  detector.update(counters, sink);  // Initialize baselines.
  for (int sample = 0; sample < 2; ++sample)
  {
    counters[0] += 201U;
    detector.update(counters, sink);
  }
  EXPECT_TRUE(events.empty());

  counters[0] += 201U;
  detector.update(counters, sink);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].channel, CanChannelId::Node1Can1);
  EXPECT_EQ(events[0].state, CanState::Fault);
  EXPECT_EQ(events[0].delta, 201U);

  detector.update(counters, sink);
  EXPECT_EQ(events.size(), 1U);
  detector.update(counters, sink);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[1].state, CanState::Normal);
}

TEST(CanErrorDetectorTest, ThresholdIsStrictlyGreaterThanConfiguredValue)
{
  CanErrorDetector detector;
  detector.configure(200U, 1U, 1U, CanErrorDetector::kAllChannelsMask);
  CanErrorDetector::CounterArray counters{};
  std::vector<CanEvent> events;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  counters[0] = 200U;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  EXPECT_TRUE(events.empty());
  counters[0] = 401U;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].state, CanState::Fault);
}

TEST(CanErrorDetectorTest, CounterResetDoesNotCreateFalseFault)
{
  CanErrorDetector detector;
  detector.configure(10U, 1U, 1U, CanErrorDetector::kAllChannelsMask);
  CanErrorDetector::CounterArray counters{};
  counters[0] = 4000000000U;
  std::vector<CanEvent> events;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  counters[0] = 3U;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(detector.state(CanChannelId::Node1Can1), CanState::Normal);
}

TEST(CanErrorDetectorTest, DisabledChannelsAreIgnored)
{
  CanErrorDetector detector;
  const std::uint16_t onlyNode1Can1 = 1U;
  detector.configure(1U, 1U, 1U, onlyNode1Can1);
  CanErrorDetector::CounterArray counters{};
  std::vector<CanEvent> events;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  counters[1] = 100U;
  detector.update(counters, [&events](const CanEvent& event) { events.push_back(event); });
  EXPECT_TRUE(events.empty());
}

TEST(CanFaultLatchTest, FaultLatchesAndRecoveryDoesNotResumeControl)
{
  CanFaultLatch latch;
  EXPECT_FALSE(latch.faulted());

  latch.update(CanEvent{10U, 3U, 2U, CanChannelId::Node2Can3, CanState::Fault});
  const std::uint16_t bit = std::uint16_t{1} <<
      static_cast<std::uint8_t>(CanChannelId::Node2Can3);
  EXPECT_TRUE(latch.faulted());
  EXPECT_EQ(latch.activeMask(), bit);
  EXPECT_EQ(latch.latchedMask(), bit);

  latch.update(CanEvent{10U, 0U, 2U, CanChannelId::Node2Can3, CanState::Normal});
  EXPECT_EQ(latch.activeMask(), 0U);
  EXPECT_EQ(latch.latchedMask(), bit);
  EXPECT_TRUE(latch.faulted());
}

TEST(CanFaultLatchTest, PreservesEveryFaultedChannel)
{
  CanFaultLatch latch;
  latch.update(CanEvent{1U, 1U, 1U, CanChannelId::Node1Can1, CanState::Fault});
  latch.update(CanEvent{1U, 1U, 1U, CanChannelId::Node3Can3, CanState::Fault});

  const std::uint16_t expected =
      (std::uint16_t{1} << static_cast<std::uint8_t>(CanChannelId::Node1Can1)) |
      (std::uint16_t{1} << static_cast<std::uint8_t>(CanChannelId::Node3Can3));
  EXPECT_EQ(latch.activeMask(), expected);
  EXPECT_EQ(latch.latchedMask(), expected);
}

TEST(CanChannelCandidateMotorsTest, ReportsCurrentPdoJointGroups)
{
  EXPECT_STREQ(CanChannelCandidateMotors(CanChannelId::Node1Can1),
               "leg_l1_joint|leg_l2_joint|leg_l3_joint");
  EXPECT_STREQ(CanChannelCandidateMotors(CanChannelId::Node2Can3),
               "arm_r4_joint");
  EXPECT_STREQ(CanChannelCandidateMotors(CanChannelId::Node3Can3),
               "waist_1_joint|waist_2_joint|waist_3_joint");
  EXPECT_STREQ(CanChannelCandidateMotors(CanChannelId::Count), "unknown");
}

}  // namespace
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
