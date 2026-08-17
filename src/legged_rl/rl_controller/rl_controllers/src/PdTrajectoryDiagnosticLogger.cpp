#include "rl_controllers/PdTrajectoryDiagnosticLogger.h"

#include <ros/package.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <ctime>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

namespace legged
{

namespace
{
constexpr const char* kLegJointNames[12] = {
    "l_leg_hip_pitch_joint", "l_leg_hip_roll_joint", "l_leg_hip_yaw_joint",
    "l_leg_knee_joint", "l_leg_ankle_pitch_joint", "l_leg_ankle_roll_joint",
    "r_leg_hip_pitch_joint", "r_leg_hip_roll_joint", "r_leg_hip_yaw_joint",
    "r_leg_knee_joint", "r_leg_ankle_pitch_joint", "r_leg_ankle_roll_joint",
};
constexpr const char* kArmJointNames[8] = {
    "l_arm_shoulder_pitch_joint", "l_arm_shoulder_roll_joint",
    "l_arm_shoulder_yaw_joint", "l_arm_elbow_joint",
    "r_arm_shoulder_pitch_joint", "r_arm_shoulder_roll_joint",
    "r_arm_shoulder_yaw_joint", "r_arm_elbow_joint",
};
constexpr const char* kWaistJointNames[3] = {
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
};
}  // namespace

PdTrajectoryDiagnosticLogger::~PdTrajectoryDiagnosticLogger()
{
  finalize("shutdown");
}

bool PdTrajectoryDiagnosticLogger::configure(ros::NodeHandle& nh,
                                             const std::vector<std::string>& control_joint_names)
{
  active_ = false;
  finalized_ = false;
  log_path_.clear();

  nh.param("/LeggedRobotCfg/pdtest/log_enabled", cfg_.enabled, false);
  nh.param("/LeggedRobotCfg/pdtest/log_duration", cfg_.duration, 0.0);
  nh.param("/LeggedRobotCfg/pdtest/sample_interval", cfg_.sample_interval, 0.002);
  nh.param("/LeggedRobotCfg/pdtest/log_prefix", cfg_.prefix, std::string("PDTest"));
  nh.param("/LeggedRobotCfg/pdtest/log_dir", cfg_.log_dir, std::string(""));

  if (cfg_.log_dir.empty())
  {
    const std::string pkg_path = ros::package::getPath("rl_controllers");
    cfg_.log_dir = pkg_path.empty() ? "/tmp/rl_logs" : (pkg_path + "/logs/run");
  }

  cfg_.duration = std::max(0.0, cfg_.duration);
  cfg_.sample_interval = std::max(1e-6, cfg_.sample_interval);

  ensureJointMetadata(control_joint_names);
  return true;
}

void PdTrajectoryDiagnosticLogger::maybeStart(const std::string& csv_input_path,
                                              double csv_sample_interval)
{
  if (!cfg_.enabled || active_ || finalized_)
  {
    return;
  }

  if (!ensureDirectory(cfg_.log_dir))
  {
    ROS_WARN_STREAM("[PdTrajectoryDiagnosticLogger] unable to prepare directory '" << cfg_.log_dir
                    << "'. Disable logging.");
    cfg_.enabled = false;
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const double timestamp_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
  const std::string timestamp = buildTimestampString(timestamp_sec);

  std::string traj_name;
  if (!csv_input_path.empty())
  {
    const size_t last_slash = csv_input_path.find_last_of("/\\");
    traj_name = (last_slash == std::string::npos) ? csv_input_path : csv_input_path.substr(last_slash + 1);
    const size_t dot = traj_name.rfind('.');
    if (dot != std::string::npos)
    {
      traj_name = traj_name.substr(0, dot);
    }
  }

  const std::string file_stem =
      (traj_name.empty() ? "" : traj_name + "_") + cfg_.prefix + "_multi_joint_" + timestamp;
  log_path_ = cfg_.log_dir + "/" + file_stem + ".csv";

  log_.open(log_path_);
  if (!log_.is_open())
  {
    ROS_ERROR_STREAM("[PdTrajectoryDiagnosticLogger] failed to open '" << log_path_ << "'.");
    return;
  }

  writeHeader(log_);
  log_time_ = 0.0;
  log_dt_ = (csv_sample_interval > 0.0) ? csv_sample_interval : cfg_.sample_interval;
  active_ = true;
  ROS_INFO_STREAM("[PdTrajectoryDiagnosticLogger] started recording to '" << log_path_ << "'.");
}

void PdTrajectoryDiagnosticLogger::writeSampleIfActive(
    std::vector<HybridJointHandle>& handles,
    const PdTrajectoryPlayer& player,
    double control_period_sec)
{
  if (!active_ || !log_.is_open())
  {
    return;
  }

  if (cfg_.duration > 0.0 && log_time_ + 1e-9 >= cfg_.duration)
  {
    finalize("log_duration reached");
    return;
  }

  writeRow(log_, log_time_, handles, player);
  log_time_ += (control_period_sec > 0.0) ? control_period_sec : log_dt_;
}

void PdTrajectoryDiagnosticLogger::finalize(const char* reason)
{
  if (!active_)
  {
    return;
  }
  if (log_.is_open())
  {
    log_.close();
  }
  ROS_INFO_STREAM("[PdTrajectoryDiagnosticLogger] closed '" << log_path_ << "'"
                  << (reason ? std::string(" (") + reason + ")" : "") << ".");
  active_ = false;
  finalized_ = true;
}

void PdTrajectoryDiagnosticLogger::resetForNextRun()
{
  finalize("reset");
  finalized_ = false;
  active_ = false;
}

bool PdTrajectoryDiagnosticLogger::ensureDirectory(const std::string& dir_path) const
{
  if (dir_path.empty())
  {
    return false;
  }

  std::string normalized = dir_path;
  while (!normalized.empty() && normalized.back() == '/')
  {
    normalized.pop_back();
  }
  if (normalized.empty())
  {
    return false;
  }

  std::string path_accumulator;
  if (normalized.front() == '/')
  {
    path_accumulator = "/";
  }

  std::stringstream ss(normalized);
  std::string segment;
  while (std::getline(ss, segment, '/'))
  {
    if (segment.empty())
    {
      continue;
    }
    if (!path_accumulator.empty() && path_accumulator.back() != '/')
    {
      path_accumulator += "/";
    }
    path_accumulator += segment;

    struct stat info
    {
    };
    if (stat(path_accumulator.c_str(), &info) != 0)
    {
      if (::mkdir(path_accumulator.c_str(), 0755) != 0 && errno != EEXIST)
      {
        ROS_ERROR_STREAM("[PdTrajectoryDiagnosticLogger] mkdir failed for '" << path_accumulator
                        << "': " << std::strerror(errno));
        return false;
      }
    }
    else if (!S_ISDIR(info.st_mode))
    {
      ROS_ERROR_STREAM("[PdTrajectoryDiagnosticLogger] path exists but is not a directory: '"
                       << path_accumulator << "'.");
      return false;
    }
  }
  return true;
}

std::string PdTrajectoryDiagnosticLogger::buildTimestampString(double wall_time_sec) const
{
  const std::time_t t = static_cast<std::time_t>(wall_time_sec);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &t);
#else
  localtime_r(&t, &tm_buf);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
  return std::string(buf);
}

void PdTrajectoryDiagnosticLogger::ensureJointMetadata(
    const std::vector<std::string>& control_joint_names)
{
  auto resolve_indices = [&](const char* const* names, size_t count,
                             std::vector<std::string>& out_names,
                             std::vector<int>& out_indices) {
    out_names.clear();
    out_indices.clear();
    for (size_t i = 0; i < count; ++i)
    {
      const std::string joint_name(names[i]);
      const auto it = std::find(control_joint_names.begin(), control_joint_names.end(), joint_name);
      if (it == control_joint_names.end())
      {
        ROS_WARN_STREAM("[PdTrajectoryDiagnosticLogger] joint not found in control_joint_names: "
                        << joint_name);
        out_indices.push_back(-1);
      }
      else
      {
        out_indices.push_back(static_cast<int>(std::distance(control_joint_names.begin(), it)));
      }
      out_names.push_back(joint_name);
    }
  };

  resolve_indices(kLegJointNames, 12, leg_joint_names_, leg_control_indices_);
  resolve_indices(kArmJointNames, 8, arm_joint_names_, arm_control_indices_);
  resolve_indices(kWaistJointNames, 3, waist_joint_names_, waist_control_indices_);
}

void PdTrajectoryDiagnosticLogger::writeGroupHeader(std::ostream& os,
                                                    const std::vector<std::string>& names) const
{
  for (const auto& name : names)
  {
    os << ",cmd_" << name << ",pos_" << name << ",cmd_vel_" << name << ",vel_" << name
       << ",torque_" << name << ",current_" << name << ",temp_" << name
       << ",acc_" << name << ",cmd_torque_" << name;
  }
}

void PdTrajectoryDiagnosticLogger::writeHeader(std::ostream& os) const
{
  os << "time";
  writeGroupHeader(os, leg_joint_names_);
  writeGroupHeader(os, arm_joint_names_);
  writeGroupHeader(os, waist_joint_names_);
  os << "\n";
}

void PdTrajectoryDiagnosticLogger::writeJointColumns(std::ostream& os,
                                                     std::vector<HybridJointHandle>& handles,
                                                     const PdTrajectoryPlayer& player,
                                                     int control_index) const
{
  if (control_index >= 0 && static_cast<size_t>(control_index) < handles.size())
  {
    auto& hj = handles[static_cast<size_t>(control_index)];
    const double cmd = player.trajOffset(static_cast<size_t>(control_index));
    const double pos = hj.getPosition();
    const double vel_des = hj.getVelocityDesired();
    const double vel = hj.getVelocity();
    const double torque = hj.getEffort();
    const double current = 0.0;
    const double temp = 0.0;
    const double acc = 0.0;
    const double pos_des = hj.getPositionDesired();
    const double kp = hj.getKp();
    const double kd = hj.getKd();
    const double ff = hj.getFeedforward();
    const double cmd_torque = kp * (pos_des - pos) + kd * (vel_des - vel) + ff;
    os << "," << cmd << "," << pos << "," << vel_des << "," << vel << "," << torque << ","
       << current << "," << temp << "," << acc << "," << cmd_torque;
  }
  else
  {
    os << ",0,0,0,0,0,0,0,0,0";
  }
}

void PdTrajectoryDiagnosticLogger::writeGroupRows(std::ostream& os,
                                                  std::vector<HybridJointHandle>& handles,
                                                  const PdTrajectoryPlayer& player,
                                                  const std::vector<int>& indices) const
{
  for (const int idx : indices)
  {
    writeJointColumns(os, handles, player, idx);
  }
}

void PdTrajectoryDiagnosticLogger::writeRow(std::ostream& os, double time_col,
                                            std::vector<HybridJointHandle>& handles,
                                            const PdTrajectoryPlayer& player)
{
  os << std::fixed << std::setprecision(9) << time_col;
  writeGroupRows(os, handles, player, leg_control_indices_);
  writeGroupRows(os, handles, player, arm_control_indices_);
  writeGroupRows(os, handles, player, waist_control_indices_);
  os << "\n";
}

}  // namespace legged
