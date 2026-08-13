#include "rl_controllers/policies/DancePolicy.h"
#include "rl_controllers/policies/WalkPolicy.h"

#include <gtest/gtest.h>

#include <type_traits>
#include <vector>

#ifndef TEST_WALK_POLICY_MODEL
#error TEST_WALK_POLICY_MODEL must be defined
#endif
#ifndef TEST_DANCE_POLICY_MODEL
#error TEST_DANCE_POLICY_MODEL must be defined
#endif
#ifndef TEST_DOWN_POLICY_MODEL
#error TEST_DOWN_POLICY_MODEL must be defined
#endif
#ifndef TEST_UP_POLICY_MODEL
#error TEST_UP_POLICY_MODEL must be defined
#endif

namespace legged
{
namespace
{
size_t elementCount(const std::vector<int64_t>& shape)
{
  size_t result = 1;
  for (const int64_t dimension : shape) result *= static_cast<size_t>(dimension);
  return result;
}

Ort::SessionOptions sessionOptions()
{
  Ort::SessionOptions options;
  options.SetInterOpNumThreads(1);
  options.SetIntraOpNumThreads(1);
  options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  return options;
}

void expectMotionPolicyExecutes(const char* path, const char* environmentName)
{
  Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, environmentName);
  DancePolicy policy;
  auto options = sessionOptions();
  policy.load(environment, path, options);
  policy.configure(23);
  ASSERT_FALSE(policy.modelInfo().inputShapes().empty());
  std::vector<float> observation(elementCount(policy.modelInfo().inputShapes()[0]), 0.0F);
  std::vector<float> actions(23, 0.0F);
  PolicyInputView input{observation.data(), observation.size()};
  PolicyOutputView output;
  output.actions = actions.data();
  output.actionCapacity = actions.size();
  Ort::RunOptions runOptions;
  EXPECT_EQ(policy.run(input, output, runOptions), PolicyRunStatus::SUCCESS);
  EXPECT_EQ(output.actionSize, 23U);
}
}  // namespace

TEST(PolicyExecutionTest, WalkOwnsAndExecutesItsModel)
{
  Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "walk_policy_test");
  WalkPolicy policy;
  auto options = sessionOptions();
  policy.load(environment, TEST_WALK_POLICY_MODEL, options);
  policy.configure(23, false);
  std::vector<float> observation(elementCount(policy.modelInfo().inputShapes()[0]), 0.0F);
  std::vector<float> actions(23, 0.0F);
  PolicyInputView input{observation.data(), observation.size()};
  PolicyOutputView output;
  output.actions = actions.data();
  output.actionCapacity = actions.size();
  Ort::RunOptions runOptions;
  EXPECT_EQ(policy.run(input, output, runOptions), PolicyRunStatus::SUCCESS);
  EXPECT_EQ(output.actionSize, 23U);
}

TEST(PolicyExecutionTest, WalkActionsRemainValidWithoutOptionalVelocityEstimate)
{
  Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "walk_policy_without_estimate_test");
  WalkPolicy policy;
  auto options = sessionOptions();
  policy.load(environment, TEST_WALK_POLICY_MODEL, options);
  policy.configure(23, true);
  std::vector<float> observation(elementCount(policy.modelInfo().inputShapes()[0]), 0.0F);
  std::vector<float> actions(23, 0.0F);
  PolicyInputView input{observation.data(), observation.size()};
  PolicyOutputView output;
  output.actions = actions.data();
  output.actionCapacity = actions.size();
  Ort::RunOptions runOptions;
  EXPECT_EQ(policy.run(input, output, runOptions),
            PolicyRunStatus::SUCCESS_WITHOUT_ESTIMATE);
  EXPECT_EQ(output.actionSize, 23U);
  EXPECT_FALSE(output.estimateValid);
}

TEST(PolicyExecutionTest, DanceOwnsAndExecutesItsModel)
{
  Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "dance_policy_test");
  DancePolicy policy;
  auto options = sessionOptions();
  policy.load(environment, TEST_DANCE_POLICY_MODEL, options);
  policy.configure(23);
  std::vector<float> observation(elementCount(policy.modelInfo().inputShapes()[0]), 0.0F);
  std::vector<float> actions(23, 0.0F);
  PolicyInputView input{observation.data(), observation.size()};
  PolicyOutputView output;
  output.actions = actions.data();
  output.actionCapacity = actions.size();
  Ort::RunOptions runOptions;
  EXPECT_EQ(policy.run(input, output, runOptions), PolicyRunStatus::SUCCESS);
  EXPECT_EQ(output.actionSize, 23U);
}

TEST(PolicyExecutionTest, WalkAndDanceOwnDistinctModels)
{
  WalkPolicy walk;
  DancePolicy dance;
  EXPECT_NE(&walk.modelInfo(), &dance.modelInfo());
}

TEST(PolicyExecutionTest, DownOwnsAndExecutesRequestedModel)
{
  expectMotionPolicyExecutes(TEST_DOWN_POLICY_MODEL, "down_policy_test");
}

TEST(PolicyExecutionTest, UpOwnsAndExecutesRequestedModel)
{
  expectMotionPolicyExecutes(TEST_UP_POLICY_MODEL, "up_policy_test");
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
