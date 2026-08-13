#include "EthercatCycleSnapshot.h"

#include <gtest/gtest.h>

#include <limits>

namespace legged
{
namespace
{

TEST(EthercatCycleTrackerTest, KeepsValidityAndWkcInOneCycleSnapshot)
{
  EthercatCycleTracker tracker;
  const auto first = tracker.observe(12, 12, 1000U, false);
  EXPECT_EQ(first.cycleSequence, 1U);
  EXPECT_TRUE(first.dataValid);
  EXPECT_EQ(first.actualWkc, 12);
  EXPECT_EQ(first.expectedWkc, 12);
  EXPECT_EQ(first.lastValidSequence, 1U);
  EXPECT_EQ(first.state, EthercatCycleState::Valid);

  const auto second = tracker.observe(11, 12, 2000U, false);
  EXPECT_EQ(second.cycleSequence, 2U);
  EXPECT_FALSE(second.dataValid);
  EXPECT_EQ(second.lastValidSequence, 1U);
  EXPECT_EQ(second.lastInvalidSequence, 2U);
  EXPECT_EQ(second.totalInvalid, 1U);
  EXPECT_EQ(second.consecutiveInvalid, 1U);
  EXPECT_EQ(second.state, EthercatCycleState::InvalidWkc);
}

TEST(EthercatCycleTrackerTest, RecoveryKeepsInvalidHistory)
{
  EthercatCycleTracker tracker;
  tracker.observe(10, 12, 1000U, false);
  tracker.observe(9, 12, 2000U, false);
  const auto recovered = tracker.observe(12, 12, 3000U, false);

  EXPECT_TRUE(recovered.dataValid);
  EXPECT_EQ(recovered.totalInvalid, 2U);
  EXPECT_EQ(recovered.lastInvalidSequence, 2U);
  EXPECT_EQ(recovered.consecutiveInvalid, 0U);
  EXPECT_EQ(recovered.lastValidSteadyTimestampNs, 3000U);
}

TEST(EthercatCycleTrackerTest, FaultLatchDoesNotFalsifyCurrentDataValidity)
{
  EthercatCycleTracker tracker;
  const auto snapshot = tracker.observe(12, 12, 1000U, true);
  EXPECT_TRUE(snapshot.dataValid);
  EXPECT_TRUE(snapshot.faultLatched);
  EXPECT_EQ(snapshot.state, EthercatCycleState::FaultLatched);
}

TEST(EthercatCycleTrackerTest, RejectsNonPositiveExpectedWkc)
{
  EthercatCycleTracker tracker;
  const auto snapshot = tracker.observe(0, 0, 1000U, false);
  EXPECT_FALSE(snapshot.dataValid);
  EXPECT_EQ(snapshot.totalInvalid, 1U);
}

}  // namespace
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
