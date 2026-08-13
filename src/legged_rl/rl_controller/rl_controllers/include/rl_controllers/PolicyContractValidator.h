#pragma once

#include "rl_controllers/OnnxPolicyModel.h"
#include "rl_controllers/PolicyMetadata.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace legged
{
class PolicyContractValidator
{
public:
  static int64_t elementCount(const std::vector<int64_t>& shape) noexcept;
  static bool metadata(const PolicyMetadata& metadata, size_t expectedDof,
                       const std::vector<std::string>& expectedJointNames,
                       std::string& error);
  static bool tensor(const OnnxPolicyModel& model, bool input, size_t index,
                     int64_t expectedElements, const std::string& label,
                     std::string& error);
};
}  // namespace legged
