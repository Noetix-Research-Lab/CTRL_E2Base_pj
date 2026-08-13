#include "rl_controllers/PolicyMetadata.h"

#include <sstream>

namespace legged
{
namespace
{
std::vector<std::string> splitStrings(const std::string& text)
{
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string value;
  while (std::getline(stream, value, ',')) values.push_back(value);
  return values;
}

std::vector<double> splitNumbers(const std::string& text)
{
  std::vector<double> values;
  std::stringstream stream(text);
  std::string value;
  while (std::getline(stream, value, ',')) {
    if (!value.empty()) values.push_back(std::stod(value));
  }
  return values;
}
}  // namespace

void PolicyMetadata::load(const OnnxPolicyModel& model)
{
  clear();
  for (const auto& entry : model.metadata()) {
    if (entry.first == "action_scale") actionScale_ = splitNumbers(entry.second);
    else if (entry.first == "joint_stiffness") stiffness_ = splitNumbers(entry.second);
    else if (entry.first == "joint_damping") damping_ = splitNumbers(entry.second);
    else if (entry.first == "default_joint_pos") defaultPosition_ = splitNumbers(entry.second);
    else if (entry.first == "joint_names") jointNames_ = splitStrings(entry.second);
  }
}

void PolicyMetadata::clear() noexcept
{
  actionScale_.clear();
  stiffness_.clear();
  damping_.clear();
  defaultPosition_.clear();
  jointNames_.clear();
}
}  // namespace legged
