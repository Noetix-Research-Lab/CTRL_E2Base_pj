#pragma once

#include "PjsolConfig.h"

#include <pjsol/pjsol.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace legged
{

struct E2MotorData;

enum class ParallelStateIssue : int
{
  None = 0,
  NonFinite,
  DriveLimit,
  SphereLimit,
  Unreachable,
  Singular,
  NotConverged,
  Other
};

const char* ParallelStateIssueName(ParallelStateIssue issue);

class ParallelJoint
{
public:
  bool init(const std::string& name, const PjsolMechanismFromYaml& cfg, std::string* error);

  void updateState(E2MotorData joints[]);
  void applyCommand(E2MotorData joints[]);

  bool ready() const { return ready_; }
  const std::string& name() const { return name_; }
  int q1Channel() const { return q1_channel_; }
  int q2Channel() const { return q2_channel_; }
  bool consumeUnreachableCommandEvent(double* qr, double* qp, uint64_t* count);
  bool consumeStateIssueEvent(ParallelStateIssue* issue, uint64_t* count, double* q1,
                              double* q2);

  pjsol::Pose2 pose() const { return x_; }
  pjsol::PoseVel2 poseVel() const { return dx_; }
  pjsol::PoseTor2 poseTor() const { return tau_x_; }
  pjsol::Joint2 joint() const { return q_; }
  pjsol::JointVel2 jointVel() const { return dq_; }
  pjsol::JointTor2 jointTor() const { return tau_q_; }

private:
  std::string name_;
  std::unique_ptr<pjsol::IMechanism> mech_;
  int q1_channel_{0};
  int q2_channel_{0};
  int q1_dir_{1};
  int q2_dir_{1};
  double q1_zero_offset_{0.0};
  double q2_zero_offset_{0.0};

  pjsol::Mat2 Jq2x_{};
  pjsol::Mat2 Jx2q_{};
  pjsol::Joint2 last_q_{};
  pjsol::Pose2 last_x_{};

  pjsol::Pose2 x_{};
  pjsol::PoseVel2 dx_{};
  pjsol::PoseTor2 tau_x_{};
  pjsol::Joint2 q_{};
  pjsol::JointVel2 dq_{};
  pjsol::JointTor2 tau_q_{};
  bool ready_{false};
  bool has_valid_state_{false};

  std::atomic_bool unreachable_cmd_event_{false};
  std::atomic<uint64_t> unreachable_cmd_count_{0};
  std::atomic<double> unreachable_cmd_qr_{0.0};
  std::atomic<double> unreachable_cmd_qp_{0.0};

  std::atomic_bool state_issue_event_{false};
  std::atomic<int> state_issue_{static_cast<int>(ParallelStateIssue::None)};
  std::atomic<uint64_t> state_issue_count_{0};
  std::atomic<double> state_issue_q1_{0.0};
  std::atomic<double> state_issue_q2_{0.0};

  void publishHeldState(E2MotorData& q1, E2MotorData& q2) const;
  void recordStateIssue(ParallelStateIssue issue, double q1, double q2);
};

}  // namespace legged
