#pragma once

#include <array>
#include <cstddef>

namespace legged
{
struct E2JointChannel
{
  std::size_t jointIndex;
  std::size_t controlGroup;
  std::size_t groupIndex;
  bool directPd;
};

// Single source of truth for the logical joint <-> ControlFSMData layout.
// Parallel ankle/waist motor-space PD is disabled because their solvers
// produce motor position/torque commands from joint-space inputs.
inline constexpr std::array<E2JointChannel, 23> kE2JointChannels{{
    {0, 0, 0, true}, {1, 0, 1, true}, {2, 0, 2, true}, {3, 0, 3, true},
    {4, 1, 0, true}, {5, 1, 1, true}, {6, 1, 2, true}, {7, 1, 3, true},
    {8, 3, 0, true}, {9, 3, 1, true}, {10, 3, 2, true}, {11, 3, 3, true},
    {12, 3, 4, false}, {13, 3, 5, false},
    {14, 2, 0, true}, {15, 2, 1, true}, {16, 2, 2, true}, {17, 2, 3, true},
    {18, 2, 4, false}, {19, 2, 5, false},
    {20, 4, 0, true}, {21, 4, 1, false}, {22, 4, 2, false},
}};
}  // namespace legged
