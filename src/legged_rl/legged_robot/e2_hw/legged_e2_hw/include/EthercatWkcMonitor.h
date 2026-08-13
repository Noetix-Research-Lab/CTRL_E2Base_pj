#pragma once

#include <cstdint>

namespace legged
{

// Fixed-window WKC error counter for the single EtherCAT RT thread.
// observe() evaluates the current sample before rolling the window so an
// error on the final cycle cannot be discarded at the boundary.
class EthercatWkcMonitor
{
public:
  constexpr EthercatWkcMonitor(std::uint32_t windowCycles,
                               std::uint32_t maxErrors) noexcept
      : windowCycles_(windowCycles == 0U ? 1U : windowCycles),
        maxErrors_(maxErrors)
  {
  }

  bool observe(bool cycleValid) noexcept
  {
    ++cycleCount_;
    if (!cycleValid)
    {
      ++errorCount_;
    }

    const bool thresholdExceeded = errorCount_ > maxErrors_;
    if (cycleCount_ >= windowCycles_)
    {
      cycleCount_ = 0U;
      errorCount_ = 0U;
    }
    return thresholdExceeded;
  }

  void reset() noexcept
  {
    cycleCount_ = 0U;
    errorCount_ = 0U;
  }

private:
  std::uint32_t windowCycles_;
  std::uint32_t maxErrors_;
  std::uint32_t cycleCount_{0U};
  std::uint32_t errorCount_{0U};
};

}  // namespace legged
