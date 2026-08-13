#include "rl_controllers/PolicyRuntimeConfig.h"

#include <algorithm>
#include <cmath>

namespace legged
{
bool PolicyRuntimeConfig::validateAndNormalize(std::string& error) noexcept
{
  error.clear();
  warmupRuns = std::max(0, warmupRuns);
  if (actionTimeoutMs < 0) {
    error = "action_timeout_ms must be non-negative (0 explicitly disables the timeout)";
    return false;
  }
  auto validFrequency = [](double value, double maximum) {
    return std::isfinite(value) && value >= 1.0 && value <= maximum;
  };
  if (!validFrequency(walkPolicyFrequencyHz, 500.0)) {
    error = "walk_policy_frequency must be finite and in [1, 500] Hz";
    return false;
  }
  if (!validFrequency(dancePolicyFrequencyHz, 500.0) ||
      !validFrequency(danceMotionFrequencyHz, 1000.0)) {
    error = "dance policy/motion frequency is outside its supported range";
    return false;
  }
  if (!std::isfinite(upTransitionDurationSec) ||
      upTransitionDurationSec < 0.1 || upTransitionDurationSec > 30.0) {
    error = "up_transition_duration must be finite and in [0.1, 30] seconds";
    return false;
  }
  if (!std::isfinite(upPolicyBlendDurationSec) ||
      upPolicyBlendDurationSec < 0.1 || upPolicyBlendDurationSec > 10.0) {
    error = "up_policy_blend_duration must be finite and in [0.1, 10] seconds";
    return false;
  }
  if (!std::isfinite(downTorqueFadeDurationSec) ||
      downTorqueFadeDurationSec < 0.1 || downTorqueFadeDurationSec > 10.0) {
    error = "down_torque_fade_duration must be finite and in [0.1, 10] seconds";
    return false;
  }
  if (inferenceThreadPriority < 0 || inferenceThreadPriority >= 95) {
    error = "inference_thread_priority must be in [0, 94]";
    return false;
  }
  if (requireRealtime && !asyncInference) {
    error = "require_realtime forbids synchronous inference";
    return false;
  }
  if (requireRealtime &&
      (inferenceCpuAffinity < 0 || inferenceThreadPriority <= 0)) {
    error = "require_realtime requires CPU affinity and positive FIFO priority";
    return false;
  }
  walkPolicyPeriodNs = static_cast<int64_t>(std::llround(1.0e9 / walkPolicyFrequencyHz));
  dancePolicyPeriodNs = static_cast<int64_t>(std::llround(1.0e9 / dancePolicyFrequencyHz));
  upTransitionDurationNs = static_cast<int64_t>(
      std::llround(upTransitionDurationSec * 1.0e9));
  upPolicyBlendDurationNs = static_cast<int64_t>(
      std::llround(upPolicyBlendDurationSec * 1.0e9));
  downTorqueFadeDurationNs = static_cast<int64_t>(
      std::llround(downTorqueFadeDurationSec * 1.0e9));
  return true;
}
}  // namespace legged
