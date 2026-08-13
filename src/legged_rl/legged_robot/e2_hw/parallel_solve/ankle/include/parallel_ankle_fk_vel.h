//
// File: parallel_ankle_fk_vel.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 29-Aug-2025 21:22:49
//
#ifndef PARALLEL_ANKLE_FK_VEL_H
#define PARALLEL_ANKLE_FK_VEL_H

// Include Files
#include "parallel_ankle_cpp_spec.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
PARALLEL_ANKLE_FD_DLL_EXPORT extern void parallel_ankle_fk_vel(double dtheta_1,
  double dtheta_2, const double jacob[4], double which_leg, double *dtheta_roll,
  double *dtheta_pitch);

#endif

//
// File trailer for parallel_ankle_fk_vel.h
//
// [EOF]
//
