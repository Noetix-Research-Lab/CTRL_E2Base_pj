#include "parallel_waist_cpp_initialize.h"
#include "parallel_waist_cpp_terminate.h"
#include "parallel_waist_fd.h"
#include "parallel_waist_fk.h"
#include "parallel_waist_fk_vel.h"
#include "parallel_waist_id.h"
#include "parallel_waist_ik.h"
#include "parallel_waist_ik_vel.h"
#include "parallel_waist_jacobian.h"

#include "parallel_ankle_cpp_initialize.h"
#include "parallel_ankle_cpp_terminate.h"
#include "parallel_ankle_fd.h"
#include "parallel_ankle_fk.h"
#include "parallel_ankle_fk_vel.h"
#include "parallel_ankle_id.h"
#include "parallel_ankle_ik.h"
#include "parallel_ankle_ik_vel.h"
#include "parallel_ankle_jacobian.h"

void parallel_waist_cpp_initialize()
{
  parallel_ankle_cpp_initialize();
}

void parallel_waist_cpp_terminate()
{
  parallel_ankle_cpp_terminate();
}

void parallel_waist_fk(double theta1, double theta2, double which_leg,
                       double* roll, double* pitch)
{
  parallel_ankle_fk(theta1, theta2, which_leg, roll, pitch);
}

void parallel_waist_ik(double theta_roll_K, double theta_pitch_K,
                       double which_leg, double is_waist_sys,
                       double* theta_1_K, double* theta_2_K)
{
  parallel_ankle_ik(theta_roll_K, theta_pitch_K, which_leg, is_waist_sys,
                    theta_1_K, theta_2_K);
}

void parallel_waist_fk_vel(double dtheta_1, double dtheta_2,
                           const double jacob[4], double which_leg,
                           double* dtheta_roll, double* dtheta_pitch)
{
  parallel_ankle_fk_vel(dtheta_1, dtheta_2, jacob, which_leg,
                        dtheta_roll, dtheta_pitch);
}

void parallel_waist_ik_vel(double theta_roll_K, double theta_pitch_K,
                           double theta_1_K, double theta_2_K,
                           double dtheta_roll_K, double dtheta_pitch_K,
                           double which_leg, double is_waist_sys,
                           double* dtheta_1_K, double* dtheta_2_K)
{
  parallel_ankle_ik_vel(theta_roll_K, theta_pitch_K, theta_1_K, theta_2_K,
                        dtheta_roll_K, dtheta_pitch_K, which_leg, is_waist_sys,
                        dtheta_1_K, dtheta_2_K);
}

void parallel_waist_jacobian(double theta_roll_K, double theta_pitch_K,
                             double theta_1_K, double theta_2_K,
                             double which_leg, double is_waist_sys,
                             double jacob_K[4])
{
  parallel_ankle_jacobian(theta_roll_K, theta_pitch_K, theta_1_K, theta_2_K,
                          which_leg, is_waist_sys, jacob_K);
}

void parallel_waist_id(double tor_roll, double tor_pitch,
                       const double jacob[4], double which_leg,
                       double* tor_1, double* tor_2)
{
  parallel_ankle_id(tor_roll, tor_pitch, jacob, which_leg, tor_1, tor_2);
}

void parallel_waist_fd(double tor_1, double tor_2, const double jacob[4],
                       double which_leg, double* tor_roll, double* tor_pitch)
{
  parallel_ankle_fd(tor_1, tor_2, jacob, which_leg, tor_roll, tor_pitch);
}
