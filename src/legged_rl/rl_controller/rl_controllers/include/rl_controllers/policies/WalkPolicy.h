#pragma once

#include "rl_controllers/policies/PolicyTypes.h"
#include "rl_controllers/OnnxPolicyModel.h"
#include "rl_controllers/PolicyMetadata.h"
#include "rl_controllers/PolicyPlaybackClock.h"

#include <cstdint>

namespace legged
{
class WalkPolicy final
{
public:
  struct RuntimeState
  {
    PolicyPlaybackClock playbackClock;
    int64_t policyPeriodNs{20000000LL};
  };

  WalkPolicy() : memoryInfo_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {}
  void load(Ort::Env& environment, const std::string& path,
            const Ort::SessionOptions& options) { model_.load(environment, path, options); metadata_.load(model_); }
  void reset() noexcept;
  bool loaded() const noexcept { return model_.loaded(); }
  const OnnxPolicyModel& modelInfo() const noexcept { return model_; }
  const PolicyMetadata& metadata() const noexcept { return metadata_; }
  RuntimeState& runtime() noexcept { return runtime_; }
  const RuntimeState& runtime() const noexcept { return runtime_; }

  void configure(size_t actionSize, bool requireVelocityEstimate);

  PolicyRunStatus run(const PolicyInputView& input, PolicyOutputView& output,
                      Ort::RunOptions& runOptions) noexcept;

private:
  size_t actionSize_{0};
  bool requireVelocityEstimate_{false};
  OnnxPolicyModel model_;
  PolicyMetadata metadata_;
  Ort::MemoryInfo memoryInfo_;
  std::vector<float> inputBuffer_;
  Ort::Value inputTensor_{nullptr};
  std::vector<std::vector<float>> outputBuffers_;
  std::vector<Ort::Value> outputTensors_;
  RuntimeState runtime_;
};
}  // namespace legged
