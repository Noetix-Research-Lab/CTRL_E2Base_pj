#pragma once

#include <array>
#include <cstddef>

namespace legged
{
struct PolicyInputView
{
  const float* observation{nullptr};
  size_t observationSize{0};
  const float* history{nullptr};
  size_t historySize{0};
  const float* command{nullptr};
  size_t commandSize{0};
};

struct PolicyOutputView
{
  float* actions{nullptr};
  size_t actionCapacity{0};
  size_t actionSize{0};
  std::array<float, 3> estimatedVelocity{{0.0F, 0.0F, 0.0F}};
  bool estimateValid{false};
};

enum class PolicyRunStatus
{
  SUCCESS,
  SUCCESS_WITHOUT_ESTIMATE,
  NOT_READY,
  INVALID_INPUT,
  INVALID_OUTPUT,
  RUNTIME_ERROR
};
}  // namespace legged

