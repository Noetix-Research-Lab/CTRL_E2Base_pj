#include "rl_controllers/policies/WalkPolicy.h"

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
      throw std::runtime_error("WalkPolicy requires static positive tensor shapes");
    }
    count *= static_cast<size_t>(dimension);
  }
  return count;
}
}  // namespace

void WalkPolicy::reset() noexcept
{
  outputTensors_.clear();
  outputBuffers_.clear();
  inputTensor_ = Ort::Value{nullptr};
  inputBuffer_.clear();
  model_.reset();
  metadata_.clear();
}

void WalkPolicy::configure(size_t actionSize, bool requireVelocityEstimate)
{
  if (!loaded() || model_.inputShapes().size() != 1 || model_.outputShapes().empty()) {
    throw std::runtime_error("WalkPolicy cannot configure tensors before loading a single-input model");
  }
  actionSize_ = actionSize;
  requireVelocityEstimate_ = requireVelocityEstimate;

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
      model_.outputNames().empty() || input.observationSize != inputBuffer_.size() ||
      outputTensors_.size() != model_.outputNames().size()) {
    return PolicyRunStatus::INVALID_INPUT;
  }
  try {
    std::copy_n(input.observation, input.observationSize, inputBuffer_.begin());
    model_.session().Run(
        runOptions, model_.inputNames().data(), &inputTensor_, 1,
        model_.outputNames().data(), outputTensors_.data(), outputTensors_.size());
    int actionIndex = model_.outputIndex("actions");
    if (actionIndex < 0) actionIndex = 0;
    if (actionIndex >= static_cast<int>(outputTensors_.size()) ||
        outputBuffers_[static_cast<size_t>(actionIndex)].size() !=
            actionSize_) {
      return PolicyRunStatus::INVALID_OUTPUT;
    }
    std::copy_n(outputBuffers_[static_cast<size_t>(actionIndex)].begin(),
                actionSize_, output.actions);
    output.actionSize = actionSize_;

    if (!requireVelocityEstimate_) return PolicyRunStatus::SUCCESS;
    const int estimateIndex = model_.outputIndex("estimate");
    if (estimateIndex < 0 || estimateIndex >= static_cast<int>(outputTensors_.size())) {
      return PolicyRunStatus::SUCCESS_WITHOUT_ESTIMATE;
    }
    if (outputBuffers_[static_cast<size_t>(estimateIndex)].size() !=
            output.estimatedVelocity.size()) {
      output.actionSize = 0;
      return PolicyRunStatus::INVALID_OUTPUT;
    }
    std::copy_n(outputBuffers_[static_cast<size_t>(estimateIndex)].begin(),
                output.estimatedVelocity.size(),
                output.estimatedVelocity.begin());
    output.estimateValid = true;
    return PolicyRunStatus::SUCCESS;
  } catch (const std::exception&) {
    output.actionSize = 0;
    return PolicyRunStatus::RUNTIME_ERROR;
  }
}
}  // namespace legged
