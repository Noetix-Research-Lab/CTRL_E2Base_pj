#pragma once

#include <cstdint>
#include <limits>

namespace legged
{

enum class EthercatCycleState : std::uint8_t
{
  Uninitialized = 0,
  Valid = 1,
  InvalidWkc = 2,
  FaultLatched = 3,
  Shutdown = 4
};

struct EthercatCycleSnapshot
{
  std::uint64_t cycleSequence{0};
  std::uint64_t lastValidSequence{0};
  std::uint64_t lastInvalidSequence{0};
  std::uint64_t steadyTimestampNs{0};
  std::uint64_t lastValidSteadyTimestampNs{0};
  std::uint64_t totalInvalid{0};
  std::uint32_t consecutiveInvalid{0};
  std::int32_t actualWkc{0};
  std::int32_t expectedWkc{0};
  EthercatCycleState state{EthercatCycleState::Uninitialized};
  bool dataValid{false};
  bool faultLatched{false};
};

// Single-threaded tracker used by the EtherCAT RT owner. Returning a complete
// value snapshot prevents diagnostics from combining WKC and validity values
// from different process-data cycles.
class EthercatCycleTracker
{
public:
  void reset() noexcept
  {
    snapshot_ = {};
  }

  const EthercatCycleSnapshot& observe(std::int32_t actualWkc,
                                       std::int32_t expectedWkc,
                                       std::uint64_t steadyTimestampNs,
                                       bool faultLatched) noexcept
  {
    ++snapshot_.cycleSequence;
    snapshot_.steadyTimestampNs = steadyTimestampNs;
    snapshot_.actualWkc = actualWkc;
    snapshot_.expectedWkc = expectedWkc;
    snapshot_.dataValid = expectedWkc > 0 && actualWkc >= expectedWkc;
    snapshot_.faultLatched = faultLatched;

    if (snapshot_.dataValid)
    {
      snapshot_.lastValidSequence = snapshot_.cycleSequence;
      snapshot_.lastValidSteadyTimestampNs = steadyTimestampNs;
      snapshot_.consecutiveInvalid = 0;
    }
    else
    {
      snapshot_.lastInvalidSequence = snapshot_.cycleSequence;
      ++snapshot_.totalInvalid;
      if (snapshot_.consecutiveInvalid !=
          std::numeric_limits<std::uint32_t>::max())
      {
        ++snapshot_.consecutiveInvalid;
      }
    }

    snapshot_.state = faultLatched
                          ? EthercatCycleState::FaultLatched
                          : (snapshot_.dataValid ? EthercatCycleState::Valid
                                                 : EthercatCycleState::InvalidWkc);
    return snapshot_;
  }

  void markShutdown(std::uint64_t steadyTimestampNs) noexcept
  {
    snapshot_.steadyTimestampNs = steadyTimestampNs;
    snapshot_.dataValid = false;
    snapshot_.state = EthercatCycleState::Shutdown;
  }

  EthercatCycleSnapshot snapshot() const noexcept
  {
    return snapshot_;
  }

private:
  EthercatCycleSnapshot snapshot_{};
};

}  // namespace legged
