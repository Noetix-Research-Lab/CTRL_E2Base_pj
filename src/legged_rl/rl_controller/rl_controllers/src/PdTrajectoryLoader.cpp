#include "rl_controllers/PdTrajectoryLoader.h"
#include "rl_controllers/JointNameAliases.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <ros/ros.h>
#include <sstream>

namespace legged
{
namespace
{

std::string trimToken(std::string token)
{
  token.erase(std::remove(token.begin(), token.end(), '\r'), token.end());
  size_t start = 0;
  while (start < token.size() && std::isspace(static_cast<unsigned char>(token[start])))
  {
    ++start;
  }
  size_t end = token.size();
  while (end > start && std::isspace(static_cast<unsigned char>(token[end - 1])))
  {
    --end;
  }
  return token.substr(start, end - start);
}

std::vector<std::string> splitCsvLine(const std::string& line)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream stream(line);
  while (std::getline(stream, token, ','))
  {
    tokens.push_back(trimToken(token));
  }
  return tokens;
}

bool parseDouble(const std::string& text, double* value)
{
  if (value == nullptr)
  {
    return false;
  }
  try
  {
    *value = std::stod(text);
    return true;
  }
  catch (...)
  {
    return false;
  }
}

}  // namespace

bool loadMultiJointTrajectoryCsv(const std::string& csv_path,
                                 const std::vector<std::string>& joint_names,
                                 bool use_csv_velocity,
                                 PdTrajectoryData& out,
                                 std::string* error_msg)
{
  auto fail = [&](const std::string& msg) {
    if (error_msg != nullptr)
    {
      *error_msg = msg;
    }
    ROS_ERROR_STREAM("[PdTrajectoryLoader] " << msg);
    return false;
  };

  out.samples.clear();
  out.vel_samples.clear();
  out.joint_from_csv.clear();
  out.sample_interval = 0.002;
  out.has_velocity = false;

  if (joint_names.empty())
  {
    return fail("joint_names is empty");
  }

  std::ifstream file(csv_path);
  if (!file.is_open())
  {
    return fail("failed to open '" + csv_path + "'");
  }

  std::string header_line;
  if (!std::getline(file, header_line))
  {
    return fail("file is empty: '" + csv_path + "'");
  }

  const std::vector<std::string> header = splitCsvLine(header_line);
  int time_idx = -1;
  std::vector<int> pos_column_indices(joint_names.size(), -1);
  std::vector<int> vel_column_indices(joint_names.size(), -1);

  for (size_t i = 0; i < header.size(); ++i)
  {
    if (header[i] == "time")
    {
      time_idx = static_cast<int>(i);
      continue;
    }
    const std::string col_name = normalizeControlJointName(header[i]);
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      if (col_name == joint_names[j])
      {
        pos_column_indices[j] = static_cast<int>(i);
      }
      else if (use_csv_velocity &&
               (col_name == ("cmd_vel_" + joint_names[j]) || col_name == ("vel_" + joint_names[j])))
      {
        vel_column_indices[j] = static_cast<int>(i);
      }
    }
  }

  if (time_idx < 0)
  {
    return fail("missing 'time' column in '" + csv_path + "'");
  }

  out.joint_from_csv.assign(joint_names.size(), false);
  int mapped_pos_cols = 0;
  for (size_t j = 0; j < pos_column_indices.size(); ++j)
  {
    if (pos_column_indices[j] >= 0)
    {
      out.joint_from_csv[j] = true;
      ++mapped_pos_cols;
    }
  }
  if (mapped_pos_cols == 0)
  {
    return fail("no joint position columns matched control_joint_names in '" + csv_path + "'");
  }

  if (use_csv_velocity)
  {
    int mapped_vel_cols = 0;
    for (int idx : vel_column_indices)
    {
      if (idx >= 0)
      {
        ++mapped_vel_cols;
      }
    }
    out.has_velocity = mapped_vel_cols > 0;
    if (!out.has_velocity)
    {
      ROS_WARN("[PdTrajectoryLoader] use_csv_velocity=true but no cmd_vel_/vel_ columns found; using zero velocity.");
    }
  }

  std::vector<double> time_stamps;
  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty())
    {
      continue;
    }
    const std::vector<std::string> cells = splitCsvLine(line);
    if (static_cast<int>(cells.size()) <= time_idx)
    {
      continue;
    }

    double t = 0.0;
    if (!parseDouble(cells[static_cast<size_t>(time_idx)], &t))
    {
      continue;
    }

    std::vector<double> sample(joint_names.size(), 0.0);
    std::vector<double> vel_sample(joint_names.size(), 0.0);
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      if (pos_column_indices[j] >= 0 &&
          pos_column_indices[j] < static_cast<int>(cells.size()))
      {
        parseDouble(cells[static_cast<size_t>(pos_column_indices[j])], &sample[j]);
      }
      if (out.has_velocity && vel_column_indices[j] >= 0 &&
          vel_column_indices[j] < static_cast<int>(cells.size()))
      {
        parseDouble(cells[static_cast<size_t>(vel_column_indices[j])], &vel_sample[j]);
      }
    }

    time_stamps.push_back(t);
    out.samples.push_back(std::move(sample));
    if (out.has_velocity)
    {
      out.vel_samples.push_back(std::move(vel_sample));
    }
  }

  if (out.samples.empty())
  {
    return fail("no valid rows in '" + csv_path + "'");
  }

  if (time_stamps.size() >= 2)
  {
    double sum_delta = 0.0;
    int valid_steps = 0;
    for (size_t i = 1; i < time_stamps.size(); ++i)
    {
      const double dt = time_stamps[i] - time_stamps[i - 1];
      if (dt > 0.0)
      {
        sum_delta += dt;
        ++valid_steps;
      }
    }
    if (valid_steps > 0)
    {
      out.sample_interval = sum_delta / static_cast<double>(valid_steps);
    }
  }

  ROS_INFO_STREAM("[PdTrajectoryLoader] loaded " << out.samples.size() << " samples from '" << csv_path
                    << "', dt=" << out.sample_interval << "s, mapped " << mapped_pos_cols << "/"
                    << joint_names.size() << " joints");
  std::ostringstream mapped;
  std::ostringstream held;
  for (size_t j = 0; j < joint_names.size(); ++j)
  {
    if (out.joint_from_csv[j])
    {
      if (!mapped.str().empty())
      {
        mapped << ", ";
      }
      mapped << joint_names[j];
    }
    else
    {
      if (!held.str().empty())
      {
        held << ", ";
      }
      held << joint_names[j];
    }
  }
  ROS_INFO_STREAM("[PdTrajectoryLoader] CSV joints: [" << mapped.str() << "]");
  ROS_INFO_STREAM("[PdTrajectoryLoader] Held joints (no CSV column): [" << held.str() << "]");
  return true;
}

}  // namespace legged
