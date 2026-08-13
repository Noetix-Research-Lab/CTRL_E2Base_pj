#include "rl_controllers/policies/DancePolicy.h"

#include <algorithm>

namespace legged
{
PolicyRunStatus DancePolicy::run(const PolicyInputView& input,
                                 PolicyOutputView& output,
                                 Ort::RunOptions& runOptions) noexcept
{
  output.actionSize = 0;
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
        model_.outputNames().data(), 1);
    if (values.empty() || !values[0].IsTensor() ||
        values[0].GetTensorTypeAndShapeInfo().GetElementCount() != actionSize_) {
      return PolicyRunStatus::INVALID_OUTPUT;
    }
    const float* actions = values[0].GetTensorData<float>();
    std::copy_n(actions, actionSize_, output.actions);
    output.actionSize = actionSize_;
    return PolicyRunStatus::SUCCESS;
  } catch (const std::exception&) {
    output.actionSize = 0;
    return PolicyRunStatus::RUNTIME_ERROR;
  }
}
}  // namespace legged
