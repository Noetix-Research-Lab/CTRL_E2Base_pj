#include "rl_controllers/PdTrajectoryPlayer.h"

#include <algorithm>
#include <ros/package.h>

namespace legged
{

bool PdTrajectoryPlayer::configure(ros::NodeHandle& nh,
                                   const std::vector<std::string>& joint_names,
                                   const PdTrajectoryControlMaps& control_maps)
{
  enabled_ = false;
  finished_ = false;
  playback_started_ = false;
  transition_active_ = false;
  sample_index_ = 0;

  default_joint_angles_ = control_maps.default_joint_angles;
  joint_stiffness_ = control_maps.joint_stiffness;
  joint_damping_ = control_maps.joint_damping;

  std::string command_mode;
  std::string csv_path;
  nh.param("/LeggedRobotCfg/pdtest/command_mode", command_mode, std::string("policy"));
  nh.param("/LeggedRobotCfg/pdtest/multi_joint_csv_path", csv_path, std::string(""));
  nh.param("/LeggedRobotCfg/pdtest/multi_joint_use_csv_desired_velocity", use_csv_velocity_, false);
  nh.param("/LeggedRobotCfg/pdtest/multi_joint_default_desired_velocity", default_desired_velocity_, 0.0);
  nh.param("/LeggedRobotCfg/pdtest/transition_duration", transition_duration_, 3.0);
  nh.param("/LeggedRobotCfg/pdtest/multi_joint_hold_missing_at_current", hold_missing_at_current_, true);

  const bool want_csv = (command_mode == "csv" || command_mode == "csv_torque") && !csv_path.empty();
  if (!want_csv)
  {
    return true;
  }
  if (command_mode == "csv_torque")
  {
    ROS_ERROR("[PdTrajectoryPlayer] csv_torque mode is not supported yet; use command_mode: csv");
    return false;
  }

  std::string resolved_path = csv_path;
  if (!resolved_path.empty() && resolved_path.front() != '/')
  {
    resolved_path = ros::package::getPath("rl_controllers") + "/" + resolved_path;
  }

  std::string error_msg;
  if (!loadMultiJointTrajectoryCsv(resolved_path, joint_names, use_csv_velocity_, trajectory_, &error_msg))
  {
    return false;
  }

  const size_t n = joint_names.size();
  joint_from_csv_ = trajectory_.joint_from_csv;
  if (joint_from_csv_.size() != n)
  {
    joint_from_csv_.assign(n, false);
  }
  held_joint_angles_.assign(n, 0.0);
  traj_offsets_.assign(n, 0.0);
  vel_des_.assign(n, default_desired_velocity_);
  pos_des_.assign(n, 0.0);
  for (size_t i = 0; i < n; ++i)
  {
    pos_des_[i] = (i < default_joint_angles_.size()) ? default_joint_angles_[i] : 0.0;
    traj_offsets_[i] = pos_des_[i];
  }

  enabled_ = true;
  resolved_csv_path_ = resolved_path;
  int csv_joint_count = 0;
  for (bool from_csv : joint_from_csv_)
  {
    if (from_csv)
    {
      ++csv_joint_count;
    }
  }
  ROS_INFO_STREAM("[PdTrajectoryPlayer] CSV trajectory enabled (absolute angles): " << resolved_path
                  << ", csv_joints=" << csv_joint_count << "/" << n
                  << ", hold_missing_at_current=" << (hold_missing_at_current_ ? "true" : "false"));
  return true;
}

void PdTrajectoryPlayer::reset()
{
  if (!enabled_)
  {
    return;
  }
  finished_ = false;
  playback_started_ = false;
  transition_active_ = false;
  sample_index_ = 0;
  transition_step_ = 0;
}

double PdTrajectoryPlayer::resolveAbsoluteCmd(size_t joint_idx, double csv_sample) const
{
  const bool from_csv =
      joint_idx < joint_from_csv_.size() && joint_from_csv_[joint_idx];
  if (from_csv)
  {
    return csv_sample;
  }
  if (hold_missing_at_current_ && joint_idx < held_joint_angles_.size())
  {
    return held_joint_angles_[joint_idx];
  }
  return (joint_idx < default_joint_angles_.size()) ? default_joint_angles_[joint_idx] : 0.0;
}

void PdTrajectoryPlayer::beginTransition(const std::vector<double>& current_pos, double control_dt)
{
  transition_active_ = true;
  transition_step_ = 0;
  transition_total_steps_ = std::max(1, static_cast<int>(transition_duration_ / control_dt));
  transition_start_pos_ = current_pos;
  transition_target_pos_.resize(current_pos.size());
  held_joint_angles_ = current_pos;

  traj_offsets_.assign(current_pos.size(), 0.0);
  vel_des_.assign(current_pos.size(), default_desired_velocity_);

  for (size_t i = 0; i < current_pos.size(); ++i)
  {
    const bool from_csv = i < joint_from_csv_.size() && joint_from_csv_[i];
    if (!trajectory_.samples.empty() && i < trajectory_.samples[0].size() && from_csv)
    {
      transition_target_pos_[i] = trajectory_.samples[0][i];
    }
    else if (hold_missing_at_current_)
    {
      transition_target_pos_[i] = current_pos[i];
    }
    else
    {
      transition_target_pos_[i] = resolveAbsoluteCmd(i, 0.0);
    }
    traj_offsets_[i] = transition_target_pos_[i];
  }
  ROS_INFO("[PdTrajectoryPlayer] transitioning CSV joints to first frame; non-CSV joints held.");
}

void PdTrajectoryPlayer::update(double control_dt, bool is_sample_step, const std::vector<double>& current_pos)
{
  if (!enabled_)
  {
    return;
  }

  if (!playback_started_ && !finished_ && !transition_active_)
  {
    if (!trajectory_.samples.empty())
    {
      beginTransition(current_pos, control_dt);
    }
  }

  if (transition_active_)
  {
    const double alpha = static_cast<double>(transition_step_) / static_cast<double>(transition_total_steps_);
    for (size_t i = 0; i < pos_des_.size(); ++i)
    {
      const bool from_csv = i < joint_from_csv_.size() && joint_from_csv_[i];
      if (!from_csv && hold_missing_at_current_)
      {
        const double held = (i < held_joint_angles_.size()) ? held_joint_angles_[i] : 0.0;
        pos_des_[i] = held;
        traj_offsets_[i] = held;
        vel_des_[i] = 0.0;
        continue;
      }
      const double start = (i < transition_start_pos_.size()) ? transition_start_pos_[i] : 0.0;
      const double target = (i < transition_target_pos_.size()) ? transition_target_pos_[i] : start;
      pos_des_[i] = start * (1.0 - alpha) + target * alpha;
      traj_offsets_[i] = pos_des_[i];
      vel_des_[i] = 0.0;
    }
    ++transition_step_;
    if (transition_step_ >= transition_total_steps_)
    {
      transition_active_ = false;
      playback_started_ = true;
      sample_index_ = 0;
      ROS_INFO("[PdTrajectoryPlayer] transition complete, playback started.");
    }
    return;
  }

  if (playback_started_ && !finished_ && is_sample_step)
  {
    if (sample_index_ < trajectory_.samples.size())
    {
      const size_t csv_idx = sample_index_;
      const auto& sample = trajectory_.samples[csv_idx];
      for (size_t i = 0; i < traj_offsets_.size(); ++i)
      {
        const double csv_sample = (i < sample.size()) ? sample[i] : 0.0;
        traj_offsets_[i] = resolveAbsoluteCmd(i, csv_sample);
        vel_des_[i] = resolveDesiredVel(static_cast<int>(i), csv_idx);
      }
      ++sample_index_;
    }
    if (sample_index_ >= trajectory_.samples.size())
    {
      finished_ = true;
      ROS_INFO("[PdTrajectoryPlayer] playback finished.");
    }
  }

  for (size_t i = 0; i < pos_des_.size(); ++i)
  {
    pos_des_[i] = (i < traj_offsets_.size()) ? traj_offsets_[i] : 0.0;
  }
}

double PdTrajectoryPlayer::resolveDesiredVel(int joint_idx, size_t sample_idx) const
{
  const bool from_csv =
      joint_idx >= 0 && static_cast<size_t>(joint_idx) < joint_from_csv_.size() &&
      joint_from_csv_[static_cast<size_t>(joint_idx)];
  if (!from_csv)
  {
    return 0.0;
  }
  if (use_csv_velocity_ && trajectory_.has_velocity &&
      sample_idx < trajectory_.vel_samples.size() &&
      joint_idx < static_cast<int>(trajectory_.vel_samples[sample_idx].size()))
  {
    return trajectory_.vel_samples[sample_idx][static_cast<size_t>(joint_idx)];
  }
  return default_desired_velocity_;
}

double PdTrajectoryPlayer::trajOffset(size_t joint_idx) const
{
  return (joint_idx < traj_offsets_.size()) ? traj_offsets_[joint_idx] : 0.0;
}

double PdTrajectoryPlayer::posDes(size_t joint_idx) const
{
  return (joint_idx < pos_des_.size()) ? pos_des_[joint_idx] : 0.0;
}

double PdTrajectoryPlayer::velDes(size_t joint_idx) const
{
  return (joint_idx < vel_des_.size()) ? vel_des_[joint_idx] : 0.0;
}

double PdTrajectoryPlayer::stiffness(size_t joint_idx) const
{
  return (joint_idx < joint_stiffness_.size()) ? joint_stiffness_[joint_idx] : 0.0;
}

double PdTrajectoryPlayer::damping(size_t joint_idx) const
{
  return (joint_idx < joint_damping_.size()) ? joint_damping_[joint_idx] : 0.0;
}

}  // namespace legged
