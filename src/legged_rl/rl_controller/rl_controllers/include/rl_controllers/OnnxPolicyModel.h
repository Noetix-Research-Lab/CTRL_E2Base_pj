#pragma once

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace legged
{
// Owns one ONNX session and all storage backing the C-string I/O names.
// Keeping this ownership in one object prevents dangling name pointers and
// makes policy model lifecycle independently testable.
class OnnxPolicyModel
{
public:
  void load(Ort::Env& environment, const std::string& path,
            const Ort::SessionOptions& options);
  void reset() noexcept;

  bool loaded() const noexcept { return session_ != nullptr; }
  Ort::Session& session() { return *session_; }
  const Ort::Session& session() const { return *session_; }
  const std::vector<const char*>& inputNames() const noexcept { return inputNames_; }
  const std::vector<const char*>& outputNames() const noexcept { return outputNames_; }
  const std::vector<std::vector<int64_t>>& inputShapes() const noexcept { return inputShapes_; }
  const std::vector<std::vector<int64_t>>& outputShapes() const noexcept { return outputShapes_; }
  const std::unordered_map<std::string, std::string>& metadata() const noexcept
  {
    return metadata_;
  }
  int outputIndex(const char* name) const noexcept;

private:
  std::unique_ptr<Ort::Session> session_;
  std::vector<Ort::AllocatedStringPtr> allocatedInputNames_;
  std::vector<Ort::AllocatedStringPtr> allocatedOutputNames_;
  std::vector<const char*> inputNames_;
  std::vector<const char*> outputNames_;
  std::vector<std::vector<int64_t>> inputShapes_;
  std::vector<std::vector<int64_t>> outputShapes_;
  std::unordered_map<std::string, std::string> metadata_;
};
}  // namespace legged
