#pragma once

#include "export.hpp"
#include "types.hpp"

namespace pjsol {

class PJSOL_API IMechanism {
public:
    virtual ~IMechanism();
    virtual const char* type() const = 0;
    virtual const JointLimit& joint_limit() const = 0;

    virtual Result ik(const Pose2& x, Joint2& q) const = 0;
    virtual Result fk(const Joint2& q, Pose2& x,
                      const Pose2& rp_init = {},
                      double timeout = 2e-4,
                      double tol = 1e-12,
                      double step_min = 1e-4) const = 0;

    virtual Result jacobian(const Pose2& x, const Joint2& q, Mat2& Jx2q, Mat2& Jq2x) const = 0;

    virtual Result vel_q2x(const Mat2& Jq2x, const JointVel2& dq, PoseVel2& dx) const = 0;
    virtual Result vel_x2q(const Mat2& Jx2q, const PoseVel2& dx, JointVel2& dq) const = 0;

    virtual Result tor_x2q(const Mat2& Jq2x, const PoseTor2& tor_x, JointTor2& tor_q) const = 0;
    virtual Result tor_q2x(const Mat2& Jx2q, const JointTor2& tor_q, PoseTor2& tor_x) const = 0;

    bool within_drive_limit(const Joint2& q) const;

    virtual bool within_joint_limit(const Pose2& x, const Joint2& q) const;
    virtual Result try_reach(const Pose2& x, Joint2& q) const;
};

} // namespace pjsol
