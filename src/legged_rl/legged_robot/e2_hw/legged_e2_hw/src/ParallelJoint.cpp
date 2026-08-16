#include "ParallelJoint.h"
#include "E2HW.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace
{

void SetError(std::string* error, const std::string& message)
{
  if (error != nullptr)
  {
    *error = message;
  }
}

bool ToVec3(const std::vector<double>& values, pjsol::Vec3& out, const char* name,
            std::string* error)
{
  if (values.size() != 3)
  {
    std::ostringstream oss;
    oss << name << " must have 3 elements, got " << values.size();
    SetError(error, oss.str());
    return false;
  }
  if (std::any_of(values.begin(), values.end(),
                  [](double v) { return !std::isfinite(v); }))
  {
    SetError(error, std::string(name) + " contains a non-finite value");
    return false;
  }
  out = {values[0], values[1], values[2]};
  return true;
}

bool IsFiniteJoint(const pjsol::Joint2& q, const pjsol::JointVel2& dq,
                   const pjsol::JointTor2& tq)
{
  return std::isfinite(q.q1) && std::isfinite(q.q2) && std::isfinite(dq.dq1) &&
         std::isfinite(dq.dq2) && std::isfinite(tq.tau1) && std::isfinite(tq.tau2);
}

legged::ParallelStateIssue FromSolveResult(pjsol::Result result)
{
  switch (result)
  {
    case pjsol::Result::Unreachable:
      return legged::ParallelStateIssue::Unreachable;
    case pjsol::Result::Singular:
      return legged::ParallelStateIssue::Singular;
    case pjsol::Result::NotConverged:
      return legged::ParallelStateIssue::NotConverged;
    default:
      return legged::ParallelStateIssue::Other;
  }
}

}  // namespace

namespace legged
{

const char* ParallelStateIssueName(ParallelStateIssue issue)
{
  switch (issue)
  {
    case ParallelStateIssue::None:
      return "none";
    case ParallelStateIssue::NonFinite:
      return "non_finite";
    case ParallelStateIssue::DriveLimit:
      return "drive_limit";
    case ParallelStateIssue::SphereLimit:
      return "sphere_limit";
    case ParallelStateIssue::Unreachable:
      return "unreachable";
    case ParallelStateIssue::Singular:
      return "singular";
    case ParallelStateIssue::NotConverged:
      return "not_converged";
    case ParallelStateIssue::Other:
      return "other";
  }
  return "other";
}

bool ParallelJoint::init(const std::string& name, const PjsolMechanismFromYaml& cfg,
                         std::string* error)
{
  ready_ = false;
  mech_.reset();
  name_ = name;
  has_valid_state_ = false;

  if (cfg.type.empty())
  {
    SetError(error, "type must not be empty");
    return false;
  }

  const auto& a1 = cfg.drive_map.q1;
  const auto& a2 = cfg.drive_map.q2;
  if (a1.channel < 0 || a1.channel >= 23 || a2.channel < 0 || a2.channel >= 23)
  {
    SetError(error, "drive_map.channel must be in [0, 22]");
    return false;
  }
  if ((a1.direction != 1 && a1.direction != -1) ||
      (a2.direction != 1 && a2.direction != -1))
  {
    SetError(error, "drive_map.direction must be +1 or -1");
    return false;
  }
  if (a1.channel == a2.channel)
  {
    SetError(error, "drive_map q1/q2 channel must differ");
    return false;
  }
  if (!std::isfinite(a1.zero_offset) || !std::isfinite(a2.zero_offset))
  {
    SetError(error, "drive_map.zero_offset is non-finite");
    return false;
  }

  q1_channel_ = a1.channel;
  q2_channel_ = a2.channel;
  q1_dir_ = a1.direction;
  q2_dir_ = a2.direction;
  q1_zero_offset_ = pjsol::math::deg2rad(a1.zero_offset);
  q2_zero_offset_ = pjsol::math::deg2rad(a2.zero_offset);

  pjsol::Geom geom{};
  if (!ToVec3(cfg.geom.F1, geom.F1, "geom.F1", error) ||
      !ToVec3(cfg.geom.F2, geom.F2, "geom.F2", error) ||
      !ToVec3(cfg.geom.H1, geom.H1, "geom.H1", error) ||
      !ToVec3(cfg.geom.H2, geom.H2, "geom.H2", error))
  {
    return false;
  }
  if (!std::isfinite(cfg.geom.l1) || !std::isfinite(cfg.geom.l2) ||
      !std::isfinite(cfg.geom.r) || !std::isfinite(cfg.geom.e) ||
      cfg.geom.l1 <= 0.0 || cfg.geom.l2 <= 0.0 || cfg.geom.r <= 0.0)
  {
    SetError(error, "geom has invalid l1/l2/r/e");
    return false;
  }
  geom.l1 = cfg.geom.l1;
  geom.l2 = cfg.geom.l2;
  geom.r = cfg.geom.r;
  geom.e = cfg.geom.e;

  if (!std::isfinite(cfg.joint_limit.q1_min) || !std::isfinite(cfg.joint_limit.q1_max) ||
      !std::isfinite(cfg.joint_limit.q2_min) || !std::isfinite(cfg.joint_limit.q2_max) ||
      !std::isfinite(cfg.joint_limit.sphere_angle_max) ||
      !(cfg.joint_limit.q1_min < cfg.joint_limit.q1_max) ||
      !(cfg.joint_limit.q2_min < cfg.joint_limit.q2_max) ||
      cfg.joint_limit.sphere_angle_max <= 0.0)
  {
    SetError(error, "joint_limit is invalid (inputs are deg)");
    return false;
  }
  pjsol::JointLimit limit{};
  limit.active.q1_min = pjsol::math::deg2rad(cfg.joint_limit.q1_min);
  limit.active.q1_max = pjsol::math::deg2rad(cfg.joint_limit.q1_max);
  limit.active.q2_min = pjsol::math::deg2rad(cfg.joint_limit.q2_min);
  limit.active.q2_max = pjsol::math::deg2rad(cfg.joint_limit.q2_max);
  limit.passive.sphere_angle_max = pjsol::math::deg2rad(cfg.joint_limit.sphere_angle_max);

  mech_ = pjsol::create(cfg.type, geom, limit);
  if (!mech_)
  {
    SetError(error, "pjsol::create failed for type='" + cfg.type + "'");
    return false;
  }

  last_q_ = {};
  last_x_ = {};
  Jq2x_ = {};
  Jx2q_ = {};
  unreachable_cmd_event_.store(false, std::memory_order_relaxed);
  unreachable_cmd_count_.store(0, std::memory_order_relaxed);
  unreachable_cmd_qr_.store(0.0, std::memory_order_relaxed);
  unreachable_cmd_qp_.store(0.0, std::memory_order_relaxed);
  state_issue_event_.store(false, std::memory_order_relaxed);
  state_issue_.store(static_cast<int>(ParallelStateIssue::None), std::memory_order_relaxed);
  state_issue_count_.store(0, std::memory_order_relaxed);
  state_issue_q1_.store(0.0, std::memory_order_relaxed);
  state_issue_q2_.store(0.0, std::memory_order_relaxed);
  return true;
}

void ParallelJoint::updateState(E2MotorData joints[])
{
  ready_ = false;
  E2MotorData& q1 = joints[q1_channel_];
  E2MotorData& q2 = joints[q2_channel_];
  const double d1 = q1_dir_;
  const double d2 = q2_dir_;

  const pjsol::Joint2 q{d1 * (q1.pos_ - q1_zero_offset_),
                        d2 * (q2.pos_ - q2_zero_offset_)};
  const pjsol::JointVel2 dq{d1 * q1.vel_, d2 * q2.vel_};
  const pjsol::JointTor2 tq{d1 * q1.tau_, d2 * q2.tau_};

  if (!mech_ || !IsFiniteJoint(q, dq, tq))
  {
    if (mech_)
    {
      recordStateIssue(ParallelStateIssue::NonFinite, q.q1, q.q2);
    }
    publishHeldState(q1, q2);
    return;
  }

  const bool over_drive = !mech_->within_drive_limit(q);

  pjsol::Pose2 x{};
  pjsol::Mat2 Jx2q{};
  pjsol::Mat2 Jq2x{};
  pjsol::PoseVel2 dx{};
  pjsol::PoseTor2 tx{};
  const pjsol::Result fk_result = mech_->fk(q, x, last_x_);
  if (fk_result != pjsol::Result::Ok)
  {
    recordStateIssue(FromSolveResult(fk_result), q.q1, q.q2);
    publishHeldState(q1, q2);
    return;
  }
  const pjsol::Result jac_result = mech_->jacobian(x, q, Jx2q, Jq2x);
  if (jac_result != pjsol::Result::Ok)
  {
    recordStateIssue(FromSolveResult(jac_result), q.q1, q.q2);
    publishHeldState(q1, q2);
    return;
  }
  if (mech_->vel_q2x(Jq2x, dq, dx) != pjsol::Result::Ok ||
      mech_->tor_q2x(Jx2q, tq, tx) != pjsol::Result::Ok)
  {
    recordStateIssue(ParallelStateIssue::Other, q.q1, q.q2);
    publishHeldState(q1, q2);
    return;
  }
  if (over_drive)
  {
    recordStateIssue(ParallelStateIssue::DriveLimit, q.q1, q.q2);
  }
  else if (!mech_->within_joint_limit(x, q))
  {
    recordStateIssue(ParallelStateIssue::SphereLimit, q.q1, q.q2);
  }

  last_q_ = q;
  last_x_ = x;
  Jq2x_ = Jq2x;
  Jx2q_ = Jx2q;
  q_ = q;
  dq_ = dq;
  tau_q_ = tq;
  x_ = x;
  dx_ = dx;
  tau_x_ = tx;
  has_valid_state_ = true;
  ready_ = true;

  q1.pos_ = x.qr;
  q1.vel_ = dx.dqr;
  q1.tau_ = tx.taur;
  q2.pos_ = x.qp;
  q2.vel_ = dx.dqp;
  q2.tau_ = tx.taup;
}

void ParallelJoint::applyCommand(E2MotorData joints[])
{
  E2MotorData& q1 = joints[q1_channel_];
  E2MotorData& q2 = joints[q2_channel_];
  if (!mech_ || (!ready_ && !has_valid_state_))
  {
    q1.ff_ = 0.0;
    q2.ff_ = 0.0;
    return;
  }

  const pjsol::Pose2 x_des{q1.pos_des_, q2.pos_des_};
  pjsol::Joint2 q_des{};
  if (mech_->try_reach(x_des, q_des) != pjsol::Result::Ok)
  {
    // RL commands can be unreachable; detect and warn only. Do not project to
    // the workspace boundary, or mapped PD torque would drop instantly.
    unreachable_cmd_qr_.store(x_des.qr, std::memory_order_relaxed);
    unreachable_cmd_qp_.store(x_des.qp, std::memory_order_relaxed);
    unreachable_cmd_count_.fetch_add(1, std::memory_order_relaxed);
    unreachable_cmd_event_.store(true, std::memory_order_release);
  }

  const pjsol::PoseTor2 tx{
      q1.kp_ * (q1.pos_des_ - q1.pos_) + q1.kd_ * (q1.vel_des_ - q1.vel_),
      q2.kp_ * (q2.pos_des_ - q2.pos_) + q2.kd_ * (q2.vel_des_ - q2.vel_)};
  pjsol::JointTor2 tq{};
  if (mech_->tor_x2q(Jq2x_, tx, tq) != pjsol::Result::Ok)
  {
    ready_ = false;
    q1.ff_ = 0.0;
    q2.ff_ = 0.0;
    return;
  }
  q1.ff_ = q1_dir_ * tq.tau1;
  q2.ff_ = q2_dir_ * tq.tau2;
}

bool ParallelJoint::consumeUnreachableCommandEvent(double* qr, double* qp, uint64_t* count)
{
  if (!unreachable_cmd_event_.exchange(false, std::memory_order_acq_rel))
  {
    return false;
  }
  if (qr != nullptr)
  {
    *qr = unreachable_cmd_qr_.load(std::memory_order_relaxed);
  }
  if (qp != nullptr)
  {
    *qp = unreachable_cmd_qp_.load(std::memory_order_relaxed);
  }
  if (count != nullptr)
  {
    *count = unreachable_cmd_count_.load(std::memory_order_relaxed);
  }
  return true;
}

void ParallelJoint::publishHeldState(E2MotorData& q1, E2MotorData& q2) const
{
  q1.pos_ = last_x_.qr;
  q1.vel_ = 0.0;
  q1.tau_ = 0.0;
  q2.pos_ = last_x_.qp;
  q2.vel_ = 0.0;
  q2.tau_ = 0.0;
}

void ParallelJoint::recordStateIssue(ParallelStateIssue issue, double q1, double q2)
{
  state_issue_.store(static_cast<int>(issue), std::memory_order_relaxed);
  state_issue_q1_.store(q1, std::memory_order_relaxed);
  state_issue_q2_.store(q2, std::memory_order_relaxed);
  state_issue_count_.fetch_add(1, std::memory_order_relaxed);
  state_issue_event_.store(true, std::memory_order_release);
}

bool ParallelJoint::consumeStateIssueEvent(ParallelStateIssue* issue, uint64_t* count,
                                           double* q1, double* q2)
{
  if (!state_issue_event_.exchange(false, std::memory_order_acq_rel))
  {
    return false;
  }
  if (issue != nullptr)
  {
    *issue = static_cast<ParallelStateIssue>(state_issue_.load(std::memory_order_relaxed));
  }
  if (count != nullptr)
  {
    *count = state_issue_count_.load(std::memory_order_relaxed);
  }
  if (q1 != nullptr)
  {
    *q1 = state_issue_q1_.load(std::memory_order_relaxed);
  }
  if (q2 != nullptr)
  {
    *q2 = state_issue_q2_.load(std::memory_order_relaxed);
  }
  return true;
}

}  // namespace legged
