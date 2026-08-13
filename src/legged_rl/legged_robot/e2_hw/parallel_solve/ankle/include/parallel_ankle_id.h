//
// File: parallel_ankle_id.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 29-Aug-2025 21:22:49
//
#ifndef PARALLEL_ANKLE_ID_H
#define PARALLEL_ANKLE_ID_H

// Include Files
#include "parallel_ankle_cpp_spec.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
PARALLEL_ANKLE_FD_DLL_EXPORT extern void parallel_ankle_id(double tor_roll,
  double tor_pitch, const double jacob[4], double which_leg, double *tor_1,
  double *tor_2);

#endif

//
// File trailer for parallel_ankle_id.h
//
// [EOF]
//
