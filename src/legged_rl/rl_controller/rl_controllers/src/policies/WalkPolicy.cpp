#include "rl_controllers/policies/WalkPolicy.h"

#include <algorithm>

namespace legged
{
PolicyRunStatus WalkPolicy::run(const PolicyInputView& input,
                                PolicyOutputView& output,
                                Ort::RunOptions& runOptions) noexcept
{
  output.actionSize = 0;
  output.estimateValid = false;
  output.estimatedVelocity = {{0.0F, 0.0F, 0.0F}};
  if (!loaded()) return PolicyRunStatus::NOT_READY;
  if (input.observation == nullptr || input.observationSize == 0 ||
      output.actions == nullptr || output.actionCapacity < actionSize_ ||
      model_.inputShapes().empty() || model_.inputNames().empty() ||
      model_.outputNames().empty()) {
    return PolicyRunStatus::INVALID_INPUT;
  }
  try {
    auto tensor = Ort::Value::CreateTensor<float>(
        memoryInfo_, const_cast<float*>(input.observation), input.observationSize,
        model_.inputShapes()[0].data(), model_.inputShapes()[0].size());
    auto values = model_.session().Run(
        runOptions, model_.inputNames().data(), &tensor, 1,
        model_.outputNames().data(), model_.outputNames().size());
    int actionIndex = model_.outputIndex("actions");
    if (actionIndex < 0) actionIndex = 0;
    if (actionIndex >= static_cast<int>(values.size()) ||
        !values[static_cast<size_t>(actionIndex)].IsTensor() ||
        values[static_cast<size_t>(actionIndex)].GetTensorTypeAndShapeInfo().GetElementCount() !=
            actionSize_) {
      return PolicyRunStatus::INVALID_OUTPUT;
    }
    const float* actions =
        values[static_cast<size_t>(actionIndex)].GetTensorData<float>();
    std::copy_n(actions, actionSize_, output.actions);
    output.actionSize = actionSize_;

    if (!requireVelocityEstimate_) return PolicyRunStatus::SUCCESS;
    const int estimateIndex = model_.outputIndex("estimate");
    if (estimateIndex < 0 || estimateIndex >= static_cast<int>(values.size())) {
      return PolicyRunStatus::SUCCESS_WITHOUT_ESTIMATE;
    }
    if (!values[static_cast<size_t>(estimateIndex)].IsTensor() ||
        values[static_cast<size_t>(estimateIndex)].GetTensorTypeAndShapeInfo().GetElementCount() !=
            output.estimatedVelocity.size()) {
      output.actionSize = 0;
      return PolicyRunStatus::INVALID_OUTPUT;
    }
    const float* estimate =
        values[static_cast<size_t>(estimateIndex)].GetTensorData<float>();
    std::copy_n(estimate, output.estimatedVelocity.size(),
                output.estimatedVelocity.begin());
    output.estimateValid = true;
    return PolicyRunStatus::SUCCESS;
  } catch (const std::exception&) {
    output.actionSize = 0;
    return PolicyRunStatus::RUNTIME_ERROR;
  }
}
}  // namespace legged
