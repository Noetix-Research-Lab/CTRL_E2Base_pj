//
// File: parallel_waist_fk_vel.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 29-Aug-2025 21:22:49
//
#ifndef PARALLEL_WAIST_FK_VEL_H
#define PARALLEL_WAIST_FK_VEL_H

// Include Files
#include "parallel_waist_cpp_spec.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
PARALLEL_WAIST_FD_DLL_EXPORT extern void parallel_waist_fk_vel(double dtheta_1,
  double dtheta_2, const double jacob[4], double which_leg, double *dtheta_roll,
  double *dtheta_pitch);

#endif

//
// File trailer for parallel_waist_fk_vel.h
//
// [EOF]
//
