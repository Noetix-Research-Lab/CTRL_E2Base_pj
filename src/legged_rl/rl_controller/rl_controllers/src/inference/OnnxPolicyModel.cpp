#include "rl_controllers/OnnxPolicyModel.h"

#include <cstring>
#include <stdexcept>

namespace legged
{
void OnnxPolicyModel::load(Ort::Env& environment, const std::string& path,
                           const Ort::SessionOptions& options)
{
  reset();
  auto session = std::make_unique<Ort::Session>(environment, path.c_str(), options);
  Ort::AllocatorWithDefaultOptions allocator;

  allocatedInputNames_.reserve(session->GetInputCount());
  inputNames_.reserve(session->GetInputCount());
  inputShapes_.reserve(session->GetInputCount());
  for (size_t index = 0; index < session->GetInputCount(); ++index) {
    allocatedInputNames_.push_back(session->GetInputNameAllocated(index, allocator));
    inputNames_.push_back(allocatedInputNames_.back().get());
    inputShapes_.push_back(
        session->GetInputTypeInfo(index).GetTensorTypeAndShapeInfo().GetShape());
  }

  allocatedOutputNames_.reserve(session->GetOutputCount());
  outputNames_.reserve(session->GetOutputCount());
  outputShapes_.reserve(session->GetOutputCount());
  for (size_t index = 0; index < session->GetOutputCount(); ++index) {
    allocatedOutputNames_.push_back(session->GetOutputNameAllocated(index, allocator));
    outputNames_.push_back(allocatedOutputNames_.back().get());
    outputShapes_.push_back(
        session->GetOutputTypeInfo(index).GetTensorTypeAndShapeInfo().GetShape());
  }

  Ort::ModelMetadata modelMetadata = session->GetModelMetadata();
  auto keys = modelMetadata.GetCustomMetadataMapKeysAllocated(allocator);
  for (auto& key : keys) {
    auto value = modelMetadata.LookupCustomMetadataMapAllocated(key.get(), allocator);
    metadata_.emplace(key.get(), value.get());
  }
  session_ = std::move(session);
}

void OnnxPolicyModel::reset() noexcept
{
  session_.reset();
  inputNames_.clear();
  outputNames_.clear();
  inputShapes_.clear();
  outputShapes_.clear();
  metadata_.clear();
  allocatedInputNames_.clear();
  allocatedOutputNames_.clear();
}

int OnnxPolicyModel::outputIndex(const char* name) const noexcept
{
  for (size_t index = 0; index < outputNames_.size(); ++index) {
    if (std::strcmp(outputNames_[index], name) == 0) return static_cast<int>(index);
  }
  return -1;
}
}  // namespace legged
