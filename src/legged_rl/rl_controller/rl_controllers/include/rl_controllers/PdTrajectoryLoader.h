#pragma once

#include <string>
#include <vector>

namespace legged
{

struct PdTrajectoryData
{
  std::vector<std::vector<double>> samples;
  std::vector<std::vector<double>> vel_samples;
  /** True if control joint j has a matched position column in the CSV header. */
  std::vector<bool> joint_from_csv;
  double sample_interval{0.002};
  bool has_velocity{false};
};

/** Load multi-joint CSV: header `time` + joint columns (URDF or E1 alias names).
 *  Matched columns are absolute joint angles; unmatched joints are filled with 0 in samples
 *  and flagged false in joint_from_csv (caller should hold standing default). */
bool loadMultiJointTrajectoryCsv(const std::string& csv_path,
                                 const std::vector<std::string>& joint_names,
                                 bool use_csv_velocity,
                                 PdTrajectoryData& out,
                                 std::string* error_msg = nullptr);

}  // namespace legged
