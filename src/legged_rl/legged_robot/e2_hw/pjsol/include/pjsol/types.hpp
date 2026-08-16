#pragma once

#include "constants.hpp"

namespace pjsol {

struct Vec3 {
    double x=0, y=0, z=0;
};

struct Mat2 {
    double a00 = 0, a01 = 0, a10 = 0, a11 = 0;
};


struct Geom {
    Vec3 F1, F2;  // 基座摇杆铰点
    Vec3 H1, H2;  // 动平台球铰点
    double l1, l2; // 连杆长度 |GiHi|
    double r;      // 摇杆臂长
    double e;      // 偏距（仅 2R 构型需要）
};

/// 主动关节限位（驱动角）
struct ActiveJointLimit {
    double q1_min = -2 * math::kPi;
    double q1_max = 2 * math::kPi;
    double q2_min = -2 * math::kPi;
    double q2_max = 2 * math::kPi;
};

/// 被动关节限位（球副等，由各构型解释）
struct PassiveJointLimit {
    double sphere_angle_max = math::deg2rad(20); // 20 deg
};

struct JointLimit {
    ActiveJointLimit active{};
    PassiveJointLimit passive{};
};

struct Pose2 {
    double qr = 0, qp = 0;
};

struct Joint2 {
    double q1 = 0, q2 = 0;
};

struct PoseVel2 {
    double dqr = 0, dqp = 0;
};

struct JointVel2 {
    double dq1 = 0, dq2 = 0;
};

struct PoseTor2 {
    double taur = 0, taup = 0;
};

struct JointTor2 {
    double tau1 = 0, tau2 = 0;
};

enum class Result {
    Ok = 0,
    InvalidArgument,
    NotConverged,
    Singular,
    Unreachable,
    UnknownType
};

} // namespace pjsol
