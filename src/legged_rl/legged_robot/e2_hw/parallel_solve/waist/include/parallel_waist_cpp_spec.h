//
// File: parallel_waist_cpp_spec.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 29-Aug-2025 21:22:49
//
#ifndef PARALLEL_WAIST_CPP_SPEC_H
#define PARALLEL_WAIST_CPP_SPEC_H

// Include Files
#ifdef PARALLEL_WAIST_FD_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define PARALLEL_WAIST_FD_DLL_EXPORT   __declspec(dllimport)
#else
#define PARALLEL_WAIST_FD_DLL_EXPORT   __attribute__ ((visibility("default")))
#endif

#elif defined(BUILDING_PARALLEL_WAIST_FD)
#if defined(_MSC_VER) || defined(__LCC__)
#define PARALLEL_WAIST_FD_DLL_EXPORT   __declspec(dllexport)
#else
#define PARALLEL_WAIST_FD_DLL_EXPORT   __attribute__ ((visibility("default")))
#endif

#else
#define PARALLEL_WAIST_FD_DLL_EXPORT
#endif
#endif

//
// File trailer for parallel_waist_cpp_spec.h
//
// [EOF]
//
