#include "rl_controllers/PolicyContractValidator.h"

#include <sstream>

namespace legged
{
namespace
{
std::string normalizeJointName(const std::string& name)
{
  return name;
}

}  // namespace

int64_t PolicyContractValidator::elementCount(const std::vector<int64_t>& shape) noexcept
{
  int64_t count = 1;
  for (const int64_t dimension : shape) {
    if (dimension < 0) return -1;
    count *= dimension;
  }
  return count;
}

bool PolicyContractValidator::metadata(
    const PolicyMetadata& value, size_t expectedDof,
    const std::vector<std::string>& expectedJointNames, std::string& error)
{
  if (value.actionScale().size() != expectedDof || value.stiffness().size() != expectedDof ||
      value.damping().size() != expectedDof || value.defaultPosition().size() != expectedDof ||
      value.jointNames().size() != expectedDof) {
    error = "metadata vector size does not match policy DOF";
    return false;
  }
  std::vector<std::string> normalizedActual;
  std::vector<std::string> normalizedExpected;
  normalizedActual.reserve(value.jointNames().size());
  normalizedExpected.reserve(expectedJointNames.size());
  for (const auto& name : value.jointNames()) {
    normalizedActual.push_back(normalizeJointName(name));
  }
  for (const auto& name : expectedJointNames) {
    normalizedExpected.push_back(normalizeJointName(name));
  }
  if (normalizedActual != normalizedExpected) {
    error = "metadata joint order does not match policy_joint_names";
    return false;
  }
  return true;
}

bool PolicyContractValidator::tensor(
    const OnnxPolicyModel& model, bool input, size_t index, int64_t expectedElements,
    const std::string& label, std::string& error)
{
  const auto& shapes = input ? model.inputShapes() : model.outputShapes();
  if (index >= shapes.size()) {
    error = "missing ONNX shape for " + label;
    return false;
  }
  const int64_t actual = elementCount(shapes[index]);
  if (actual > 0 && actual != expectedElements) {
    std::ostringstream stream;
    stream << label << " has " << actual << " elements, expected " << expectedElements;
    error = stream.str();
    return false;
  }
  return true;
}

}  // namespace legged
