#include "rl_controllers/PolicyPlaybackClock.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace legged
{
TEST(PolicyPlaybackClockTest, RejectsInvalidAndEarlyTicks)
{
  PolicyPlaybackClock clock;
  uint64_t tick = 99;
  EXPECT_FALSE(clock.take(20, 100, tick));
  clock.reset(100);
  EXPECT_FALSE(clock.take(0, 100, tick));
  ASSERT_TRUE(clock.take(20, 100, tick));
  EXPECT_EQ(tick, 0U);
  EXPECT_FALSE(clock.take(20, 119, tick));
  EXPECT_FALSE(clock.take(20, 100, tick));
}

TEST(PolicyPlaybackClockTest, SkipsLateTicksWithoutPhaseDrift)
{
  PolicyPlaybackClock clock;
  constexpr int64_t kStartNs = 1'000'000;
  constexpr int64_t kPeriodNs = 20'000;
  clock.reset(kStartNs);

  uint64_t tick = 0;
  ASSERT_TRUE(clock.take(kPeriodNs, kStartNs, tick));
  EXPECT_EQ(tick, 0U);
  ASSERT_TRUE(clock.take(kPeriodNs, kStartNs + 7 * kPeriodNs + 1, tick));
  EXPECT_EQ(tick, 7U);
  EXPECT_EQ(clock.nextPolicyNs(), kStartNs + 8 * kPeriodNs);
  EXPECT_FALSE(clock.take(kPeriodNs, kStartNs + 8 * kPeriodNs - 1, tick));
  ASSERT_TRUE(clock.take(kPeriodNs, kStartNs + 8 * kPeriodNs, tick));
  EXPECT_EQ(tick, 8U);
}

TEST(PolicyPlaybackClockTest, SustainsMillionsOfControlCycles)
{
  PolicyPlaybackClock clock;
  constexpr int64_t kStartNs = 10'000;
  constexpr int64_t kControlPeriodNs = 2'000'000;
  constexpr int64_t kPolicyPeriodNs = 20'000'000;
  constexpr uint64_t kControlCycles = 2'000'000;
  clock.reset(kStartNs);

  uint64_t accepted = 0;
  uint64_t tick = 0;
  for (uint64_t cycle = 0; cycle < kControlCycles; ++cycle) {
    if (clock.take(kPolicyPeriodNs,
                   kStartNs + static_cast<int64_t>(cycle) * kControlPeriodNs,
                   tick)) {
      EXPECT_EQ(tick, cycle / 10);
      ++accepted;
    }
  }
  EXPECT_EQ(accepted, kControlCycles / 10);
}

TEST(PolicyPlaybackClockTest, SaturatesAtNanosecondLimit)
{
  constexpr int64_t kMaxNs = std::numeric_limits<int64_t>::max();
  PolicyPlaybackClock clock;
  clock.reset(kMaxNs - 100);
  uint64_t tick = 0;
  ASSERT_TRUE(clock.take(20, kMaxNs - 100, tick));
  ASSERT_TRUE(clock.take(20, kMaxNs, tick));
  EXPECT_EQ(tick, 5U);
  EXPECT_EQ(clock.nextPolicyNs(), kMaxNs);
  EXPECT_FALSE(clock.take(20, kMaxNs, tick));
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
