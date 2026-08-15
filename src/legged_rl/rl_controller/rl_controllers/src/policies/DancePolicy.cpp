#include "rl_controllers/policies/DancePolicy.h"

#include <algorithm>
#include <stdexcept>

namespace legged
{
namespace
{
size_t staticElementCount(const std::vector<int64_t>& shape)
{
  size_t count = 1;
  for (const int64_t dimension : shape) {
    if (dimension <= 0) {
      throw std::runtime_error("DancePolicy requires static positive tensor shapes");
    }
    count *= static_cast<size_t>(dimension);
  }
  return count;
}
}  // namespace

void DancePolicy::reset() noexcept
{
  outputTensors_.clear();
  outputBuffers_.clear();
  inputTensor_ = Ort::Value{nullptr};
  inputBuffer_.clear();
  model_.reset();
  metadata_.clear();
}

void DancePolicy::configure(size_t actionSize)
{
  if (!loaded() || model_.inputShapes().size() != 1 || model_.outputShapes().empty()) {
    throw std::runtime_error("DancePolicy cannot configure tensors before loading a single-input model");
  }
  actionSize_ = actionSize;
  runtime_.referenceJointPosition.setZero(actionSize);
  runtime_.referenceJointVelocity.setZero(actionSize);

  inputTensor_ = Ort::Value{nullptr};
  inputBuffer_.assign(staticElementCount(model_.inputShapes()[0]), 0.0F);
  inputTensor_ = Ort::Value::CreateTensor<float>(
      memoryInfo_, inputBuffer_.data(), inputBuffer_.size(),
      model_.inputShapes()[0].data(), model_.inputShapes()[0].size());

  outputTensors_.clear();
  outputBuffers_.clear();
  outputBuffers_.resize(model_.outputShapes().size());
  outputTensors_.reserve(model_.outputShapes().size());
  for (size_t index = 0; index < model_.outputShapes().size(); ++index) {
    const auto& shape = model_.outputShapes()[index];
    auto& buffer = outputBuffers_[index];
    buffer.assign(staticElementCount(shape), 0.0F);
    outputTensors_.emplace_back(Ort::Value::CreateTensor<float>(
        memoryInfo_, buffer.data(), buffer.size(), shape.data(), shape.size()));
  }
}

PolicyRunStatus DancePolicy::run(const PolicyInputView& input,
                                 PolicyOutputView& output,
                                 Ort::RunOptions& runOptions) noexcept
{
  output.actionSize = 0;
  if (!loaded()) return PolicyRunStatus::NOT_READY;
  if (input.observation == nullptr || input.observationSize == 0 ||
      output.actions == nullptr || output.actionCapacity < actionSize_ ||
      model_.inputShapes().empty() || model_.inputNames().empty() ||
      model_.outputNames().empty() || input.observationSize != inputBuffer_.size() ||
      outputTensors_.empty()) {
    return PolicyRunStatus::INVALID_INPUT;
  }
  try {
    std::copy_n(input.observation, input.observationSize, inputBuffer_.begin());
    model_.session().Run(
        runOptions, model_.inputNames().data(), &inputTensor_, 1,
        model_.outputNames().data(), outputTensors_.data(), 1);
    if (outputBuffers_[0].size() != actionSize_) {
      return PolicyRunStatus::INVALID_OUTPUT;
    }
    std::copy_n(outputBuffers_[0].begin(), actionSize_, output.actions);
    output.actionSize = actionSize_;
    return PolicyRunStatus::SUCCESS;
  } catch (const std::exception&) {
    output.actionSize = 0;
    return PolicyRunStatus::RUNTIME_ERROR;
  }
}
}  // namespace legged
