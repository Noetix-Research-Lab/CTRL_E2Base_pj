#include "rl_controllers/OnnxPolicyModel.h"
#include "rl_controllers/PolicyContractValidator.h"
#include "rl_controllers/PolicyMetadata.h"

#include <gtest/gtest.h>

#include <string>

#ifndef TEST_POLICY_MODEL
#error TEST_POLICY_MODEL must name a policy model used by the lifecycle test
#endif

namespace legged
{
TEST(OnnxPolicyModelTest, LoadsMetadataAndKeepsIoNamesAlive)
{
  Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "onnx_model_test");
  Ort::SessionOptions options;
  options.SetInterOpNumThreads(1);
  options.SetIntraOpNumThreads(1);
  options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);

  OnnxPolicyModel model;
  model.load(environment, TEST_POLICY_MODEL, options);
  ASSERT_TRUE(model.loaded());
  ASSERT_FALSE(model.inputNames().empty());
  ASSERT_FALSE(model.outputNames().empty());
  EXPECT_EQ(model.inputNames().size(), model.inputShapes().size());
  EXPECT_EQ(model.outputNames().size(), model.outputShapes().size());
  EXPECT_FALSE(model.metadata().at("joint_names").empty());

  // Repeated reset/load catches ownership bugs in AllocatedStringPtr-backed
  // names and provides a useful ASan/LSan lifecycle workload.
  for (int iteration = 0; iteration < 10; ++iteration) {
    model.reset();
    EXPECT_FALSE(model.loaded());
    model.load(environment, TEST_POLICY_MODEL, options);
    ASSERT_TRUE(model.loaded());
    EXPECT_NE(model.inputNames().front(), nullptr);
  }
}

TEST(OnnxPolicyModelTest, MetadataAndTensorContractsAreValidatedOutsideController)
{
  Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "policy_contract_test");
  Ort::SessionOptions options;
  options.SetInterOpNumThreads(1);
  options.SetIntraOpNumThreads(1);
  OnnxPolicyModel model;
  model.load(environment, TEST_POLICY_MODEL, options);

  PolicyMetadata metadata;
  metadata.load(model);
  std::string error;
  ASSERT_FALSE(metadata.jointNames().empty());
  EXPECT_TRUE(PolicyContractValidator::metadata(
      metadata, metadata.jointNames().size(), metadata.jointNames(), error)) << error;
  const int64_t inputElements =
      PolicyContractValidator::elementCount(model.inputShapes().front());
  EXPECT_TRUE(PolicyContractValidator::tensor(
      model, true, 0, inputElements, "test input", error)) << error;
  EXPECT_FALSE(PolicyContractValidator::tensor(
      model, true, 0, inputElements + 1, "test input", error));
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
