#pragma once

#include "rl_controllers/PdTrajectoryPlayer.h"

#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <fstream>
#include <string>
#include <vector>

namespace legged
{

struct PdTrajectoryLogConfig
{
  bool enabled{false};
  double duration{0.0};
  double sample_interval{0.002};
  std::string log_dir;
  std::string prefix{"PDTest"};
};

/** Multi-joint diagnostic CSV logger (leg + arm + waist columns). */
class PdTrajectoryDiagnosticLogger
{
public:
  bool configure(ros::NodeHandle& nh, const std::vector<std::string>& control_joint_names);
  bool loggingEnabled() const { return cfg_.enabled; }
  void maybeStart(const std::string& csv_input_path, double csv_sample_interval);
  void writeSampleIfActive(std::vector<HybridJointHandle>& handles,
                           const PdTrajectoryPlayer& player,
                           double control_period_sec = 0.002);
  void finalize(const char* reason);
  void resetForNextRun();
  ~PdTrajectoryDiagnosticLogger();

private:
  bool ensureDirectory(const std::string& dir_path) const;
  std::string buildTimestampString(double wall_time_sec) const;
  void ensureJointMetadata(const std::vector<std::string>& control_joint_names);
  void writeHeader(std::ostream& os) const;
  void writeRow(std::ostream& os, double time_col,
                std::vector<HybridJointHandle>& handles,
                const PdTrajectoryPlayer& player);
  void writeJointColumns(std::ostream& os,
                         std::vector<HybridJointHandle>& handles,
                         const PdTrajectoryPlayer& player,
                         int control_index) const;
  void writeGroupHeader(std::ostream& os, const std::vector<std::string>& names) const;
  void writeGroupRows(std::ostream& os,
                     std::vector<HybridJointHandle>& handles,
                     const PdTrajectoryPlayer& player,
                     const std::vector<int>& indices) const;

  PdTrajectoryLogConfig cfg_;
  std::vector<std::string> leg_joint_names_;
  std::vector<int> leg_control_indices_;
  std::vector<std::string> arm_joint_names_;
  std::vector<int> arm_control_indices_;
  std::vector<std::string> waist_joint_names_;
  std::vector<int> waist_control_indices_;

  bool active_{false};
  bool finalized_{false};
  std::ofstream log_;
  std::string log_path_;
  double log_time_{0.0};
  double log_dt_{0.002};
};

}  // namespace legged
