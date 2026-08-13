#pragma once

#include "rl_controllers/OnnxPolicyModel.h"

#include <string>
#include <vector>

namespace legged
{
class PolicyMetadata
{
public:
  void load(const OnnxPolicyModel& model);
  void clear() noexcept;

  const std::vector<double>& actionScale() const noexcept { return actionScale_; }
  const std::vector<double>& stiffness() const noexcept { return stiffness_; }
  const std::vector<double>& damping() const noexcept { return damping_; }
  const std::vector<double>& defaultPosition() const noexcept { return defaultPosition_; }
  const std::vector<std::string>& jointNames() const noexcept { return jointNames_; }

private:
  std::vector<double> actionScale_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> defaultPosition_;
  std::vector<std::string> jointNames_;
};
}  // namespace legged
