#pragma once

#include "rl_controllers/PdTrajectoryLoader.h"

#include <ros/ros.h>
#include <string>
#include <vector>

namespace legged
{

/** Per-joint PD gains and default angles in control_joint_names order. */
struct PdTrajectoryControlMaps
{
  std::vector<double> default_joint_angles;
  std::vector<double> joint_stiffness;
  std::vector<double> joint_damping;
};

/**
 * CSV joint-trajectory player: transition to first frame, sample playback, hold last frame.
 * CSV columns are absolute joint angles. Joints missing from CSV can hold the pose captured
 * at playback start (recommended for arm-only / leg-only sparse CSV).
 */
class PdTrajectoryPlayer
{
public:
  bool configure(ros::NodeHandle& nh,
                 const std::vector<std::string>& joint_names,
                 const PdTrajectoryControlMaps& control_maps);

  bool enabled() const { return enabled_; }
  bool finished() const { return finished_; }
  bool playbackStarted() const { return playback_started_; }

  /** Resolved path passed to PdTrajectoryLoader. */
  const std::string& resolvedCsvPath() const { return resolved_csv_path_; }
  double sampleInterval() const { return trajectory_.sample_interval; }
  /** Absolute pos_des source (CSV angle, held pose, or standing default). */
  double trajOffset(size_t joint_idx) const;

  /** Advance state machine; call each control cycle when enabled(). */
  void update(double control_dt, bool is_sample_step, const std::vector<double>& current_pos);

  /** Restart transition + playback from the first CSV frame. */
  void reset();

  size_t numJoints() const { return pos_des_.size(); }
  double posDes(size_t joint_idx) const;
  double velDes(size_t joint_idx) const;
  double stiffness(size_t joint_idx) const;
  double damping(size_t joint_idx) const;

private:
  double resolveDesiredVel(int joint_idx, size_t sample_idx) const;
  void beginTransition(const std::vector<double>& current_pos, double control_dt);
  double resolveAbsoluteCmd(size_t joint_idx, double csv_sample) const;

  bool enabled_{false};
  bool finished_{false};
  bool playback_started_{false};
  bool transition_active_{false};
  bool hold_missing_at_current_{true};

  double transition_duration_{3.0};
  double default_desired_velocity_{0.0};
  bool use_csv_velocity_{false};

  PdTrajectoryData trajectory_;
  size_t sample_index_{0};
  int transition_step_{0};
  int transition_total_steps_{1};

  std::vector<double> default_joint_angles_;
  std::vector<double> joint_stiffness_;
  std::vector<double> joint_damping_;
  std::vector<bool> joint_from_csv_;
  /** Snapshot at playback start for joints absent from CSV when hold_missing_at_current_. */
  std::vector<double> held_joint_angles_;
  std::vector<double> transition_start_pos_;
  std::vector<double> transition_target_pos_;
  std::vector<double> traj_offsets_;
  std::vector<double> vel_des_;
  std::vector<double> pos_des_;
  std::string resolved_csv_path_;
};

}  // namespace legged
