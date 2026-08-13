#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace legged
{
enum class PolicyKind : uint8_t
{
  NONE,
  WALK,
  DANCE,
  DOWN,
  UP
};

struct ObservationPacket
{
  PolicyKind policy{PolicyKind::NONE};
  uint64_t epoch{0};
  uint64_t sequence{0};
  int64_t timestampNs{0};
  size_t size{0};
  std::vector<float> data;
  size_t historySize{0};
  size_t commandSize{0};
  std::vector<float> history;
  std::vector<float> command;
};

struct ActionPacket
{
  PolicyKind policy{PolicyKind::NONE};
  uint64_t epoch{0};
  uint64_t sourceSequence{0};
  int64_t observationTimestampNs{0};
  int64_t completionTimestampNs{0};
  uint32_t inferenceTimeUs{0};
  size_t size{0};
  bool valid{false};
  std::vector<float> actions;
  std::array<float, 3> estimatedVelocity{{0.0F, 0.0F, 0.0F}};
  bool estimateValid{false};
};

struct TracePacket
{
  size_t observationSize{0};
  size_t actionSize{0};
  std::vector<float> observation;
  std::vector<float> action;
  size_t historySize{0};
  size_t commandSize{0};
  std::vector<float> history;
  std::vector<float> command;
};

struct InferenceTimingSample
{
  uint64_t sequence{0};
  double policy{0.0};
  uint64_t epoch{0};
  int64_t observationNs{0};
  int64_t startNs{0};
  int64_t completionNs{0};
  double inferenceUs{0.0};
  double queueUs{0.0};
  double endToEndUs{0.0};
  double success{0.0};
  double asyncMode{0.0};
};

struct ActionTimingSample
{
  uint64_t sequence{0};
  double policy{0.0};
  uint64_t epoch{0};
  int64_t observationNs{0};
  int64_t completionNs{0};
  int64_t consumedNs{0};
  double completionToConsumeUs{0.0};
  double observationToConsumeUs{0.0};
  double asyncMode{0.0};
};
}  // namespace legged
