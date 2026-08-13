#include "rl_controllers/PolicyRuntimeConfig.h"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace legged
{
TEST(PolicyRuntimeConfigTest, NormalizesSafeBoundsAndComputesPeriods)
{
  PolicyRuntimeConfig config;
  config.actionTimeoutMs = 0;
  config.warmupRuns = -5;
  config.walkPolicyFrequencyHz = 100.0;
  std::string error;
  ASSERT_TRUE(config.validateAndNormalize(error)) << error;
  EXPECT_EQ(config.actionTimeoutMs, 0);
  EXPECT_EQ(config.warmupRuns, 0);
  EXPECT_EQ(config.walkPolicyPeriodNs, 10000000LL);
  EXPECT_EQ(config.upTransitionDurationNs, 2000000000LL);
  EXPECT_EQ(config.upPolicyBlendDurationNs, 500000000LL);
  EXPECT_EQ(config.downTorqueFadeDurationNs, 1000000000LL);
}

TEST(PolicyRuntimeConfigTest, ValidatesDownTorqueFadeDuration)
{
  PolicyRuntimeConfig config;
  std::string error;

  config.downTorqueFadeDurationSec = 1.5;
  ASSERT_TRUE(config.validateAndNormalize(error)) << error;
  EXPECT_EQ(config.downTorqueFadeDurationNs, 1500000000LL);

  config.downTorqueFadeDurationSec = 0.0;
  EXPECT_FALSE(config.validateAndNormalize(error));
  EXPECT_NE(error.find("down_torque_fade_duration"), std::string::npos);
}

TEST(PolicyRuntimeConfigTest, ValidatesUpPolicyBlendDuration)
{
  PolicyRuntimeConfig config;
  std::string error;

  config.upPolicyBlendDurationSec = 0.75;
  ASSERT_TRUE(config.validateAndNormalize(error)) << error;
  EXPECT_EQ(config.upPolicyBlendDurationNs, 750000000LL);

  config.upPolicyBlendDurationSec = 0.0;
  EXPECT_FALSE(config.validateAndNormalize(error));
  EXPECT_NE(error.find("up_policy_blend_duration"), std::string::npos);
}

TEST(PolicyRuntimeConfigTest, ValidatesUpTransitionDuration)
{
  PolicyRuntimeConfig config;
  std::string error;

  config.upTransitionDurationSec = 1.5;
  ASSERT_TRUE(config.validateAndNormalize(error)) << error;
  EXPECT_EQ(config.upTransitionDurationNs, 1500000000LL);

  config.upTransitionDurationSec = 0.0;
  EXPECT_FALSE(config.validateAndNormalize(error));
  EXPECT_NE(error.find("up_transition_duration"), std::string::npos);
}

TEST(PolicyRuntimeConfigTest, RejectsNegativeActionTimeout)
{
  PolicyRuntimeConfig config;
  config.actionTimeoutMs = -1;
  std::string error;

  EXPECT_FALSE(config.validateAndNormalize(error));
  EXPECT_NE(error.find("action_timeout_ms"), std::string::npos);
}

TEST(PolicyRuntimeConfigTest, RejectsNonFiniteAndUnsafeRealtimeSettings)
{
  PolicyRuntimeConfig config;
  config.walkPolicyFrequencyHz = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  EXPECT_FALSE(config.validateAndNormalize(error));

  config = PolicyRuntimeConfig{};
  config.requireRealtime = true;
  config.asyncInference = false;
  EXPECT_FALSE(config.validateAndNormalize(error));

  config.asyncInference = true;
  config.inferenceCpuAffinity = -1;
  EXPECT_FALSE(config.validateAndNormalize(error));
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
