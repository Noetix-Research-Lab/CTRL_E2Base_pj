#pragma once

#include <cstdint>
#include <string>

namespace legged
{
struct PolicyRuntimeConfig
{
  int actionTimeoutMs{100};
  int inferenceCpuAffinity{-1};
  int inferenceThreadPriority{20};
  int warmupRuns{20};
  bool asyncInference{true};
  bool enableTimingMetrics{false};
  bool requireRealtime{false};
  double walkPolicyFrequencyHz{50.0};
  double dancePolicyFrequencyHz{50.0};
  double danceMotionFrequencyHz{50.0};
  double upTransitionDurationSec{2.0};
  double upPolicyBlendDurationSec{0.5};
  double downTorqueFadeDurationSec{1.0};

  int64_t walkPolicyPeriodNs{20000000LL};
  int64_t dancePolicyPeriodNs{20000000LL};
  int64_t upTransitionDurationNs{2000000000LL};
  int64_t upPolicyBlendDurationNs{500000000LL};
  int64_t downTorqueFadeDurationNs{1000000000LL};

  bool validateAndNormalize(std::string& error) noexcept;
};
}  // namespace legged
