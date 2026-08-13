#include "EthercatWkcMonitor.h"

#include <gtest/gtest.h>

namespace legged
{
namespace
{

TEST(EthercatWkcMonitorTest, FaultsOnTwentyFirstError)
{
  EthercatWkcMonitor monitor(100U, 20U);
  for (int error = 0; error < 20; ++error)
  {
    EXPECT_FALSE(monitor.observe(false));
  }
  EXPECT_TRUE(monitor.observe(false));
}

TEST(EthercatWkcMonitorTest, FinalWindowCycleCannotHideThresholdCrossing)
{
  EthercatWkcMonitor monitor(100U, 20U);
  for (int cycle = 0; cycle < 79; ++cycle)
  {
    EXPECT_FALSE(monitor.observe(true));
  }
  for (int error = 0; error < 20; ++error)
  {
    EXPECT_FALSE(monitor.observe(false));
  }

  // This is both the 100th sample and the 21st error. The old implementation
  // cleared the counters before evaluating this boundary sample.
  EXPECT_TRUE(monitor.observe(false));
}

TEST(EthercatWkcMonitorTest, RollsToAnIndependentNextWindow)
{
  EthercatWkcMonitor monitor(4U, 1U);
  EXPECT_FALSE(monitor.observe(false));
  EXPECT_FALSE(monitor.observe(true));
  EXPECT_FALSE(monitor.observe(true));
  EXPECT_FALSE(monitor.observe(true));

  EXPECT_FALSE(monitor.observe(false));
  EXPECT_TRUE(monitor.observe(false));
}

}  // namespace
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
