#ifndef LEGGED_E2_HW_PVT_COMMAND_SAFETY_H
#define LEGGED_E2_HW_PVT_COMMAND_SAFETY_H

#include <cmath>

struct PvtCommandValues {
  float kp;
  float kd;
  float pos;
  float spd;
  float tor;
};

// Final non-allocating finite-value barrier before motor commands reach the
// PDO encoder. The logical zero command is safe to encode and transmit.
inline bool SanitizePvtCommandValues(PvtCommandValues& command) noexcept
{
  const bool valid = std::isfinite(command.kp) && std::isfinite(command.kd) &&
                     std::isfinite(command.pos) && std::isfinite(command.spd) &&
                     std::isfinite(command.tor);
  if (!valid) {
    command = PvtCommandValues{0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  }
  return valid;
}

#endif  // LEGGED_E2_HW_PVT_COMMAND_SAFETY_H
