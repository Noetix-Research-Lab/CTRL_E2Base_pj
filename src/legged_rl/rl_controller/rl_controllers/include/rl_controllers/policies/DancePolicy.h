#pragma once

#include "rl_controllers/policies/PolicyTypes.h"
#include "rl_controllers/OnnxPolicyModel.h"
#include "rl_controllers/PolicyMetadata.h"
#include "rl_controllers/PolicyPlaybackClock.h"
#include "rl_controllers/Types.h"

#include <cstdint>
#include <vector>

namespace legged
{
class DancePolicy final
{
public:
  struct RuntimeState
  {
    PolicyPlaybackClock playbackClock;
    int64_t policyPeriodNs{20000000LL};
    double motionFrequencyHz{50.0};
    bool endPending{false};
    uint64_t finalObservationSequence{0};
    int64_t completionNotBeforeNs{0};
    uint32_t alignmentSamples{0};
    vector_t referenceJointPosition;
    vector_t referenceJointVelocity;
    Eigen::Quaterniond initialToWorld{Eigen::Quaterniond::Identity()};
    std::vector<std::vector<double>> jointPositionFrames;
    std::vector<std::vector<double>> jointVelocityFrames;
    std::vector<std::vector<double>> bodyQuaternionFrames;
    size_t timestepCount{0};
    size_t jointCount{0};
  };

  DancePolicy() : memoryInfo_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {}
  void load(Ort::Env& environment, const std::string& path,
            const Ort::SessionOptions& options) { model_.load(environment, path, options); metadata_.load(model_); }
  void reset() noexcept { model_.reset(); metadata_.clear(); }
  bool loaded() const noexcept { return model_.loaded(); }
  const OnnxPolicyModel& modelInfo() const noexcept { return model_; }
  const PolicyMetadata& metadata() const noexcept { return metadata_; }
  RuntimeState& runtime() noexcept { return runtime_; }
  const RuntimeState& runtime() const noexcept { return runtime_; }

  void configure(size_t actionSize)
  {
    actionSize_ = actionSize;
    runtime_.referenceJointPosition.setZero(actionSize);
    runtime_.referenceJointVelocity.setZero(actionSize);
  }

  PolicyRunStatus run(const PolicyInputView& input, PolicyOutputView& output,
                      Ort::RunOptions& runOptions) noexcept;

private:
  size_t actionSize_{0};
  OnnxPolicyModel model_;
  PolicyMetadata metadata_;
  Ort::MemoryInfo memoryInfo_;
  RuntimeState runtime_;
};
}  // namespace legged
