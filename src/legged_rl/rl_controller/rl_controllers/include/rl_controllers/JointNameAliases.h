#pragma once

#include <string>
#include <unordered_map>

namespace legged
{

/** Map E1 / legacy CSV column names to E2 URDF control_joint_names. */
inline std::string normalizeControlJointName(const std::string& name)
{
  static const std::unordered_map<std::string, std::string> aliases{
      {"leg_l1_joint", "l_leg_hip_yaw_joint"},
      {"leg_r1_joint", "r_leg_hip_yaw_joint"},
      {"waist_1_joint", "waist_yaw_joint"},
      {"leg_l2_joint", "l_leg_hip_roll_joint"},
      {"leg_r2_joint", "r_leg_hip_roll_joint"},
      {"waist_2_joint", "waist_roll_joint"},
      {"leg_l3_joint", "l_leg_hip_pitch_joint"},
      {"leg_r3_joint", "r_leg_hip_pitch_joint"},
      {"leg_l4_joint", "l_leg_knee_joint"},
      {"leg_r4_joint", "r_leg_knee_joint"},
      {"leg_l5_joint", "l_leg_ankle_pitch_joint"},
      {"leg_r5_joint", "r_leg_ankle_pitch_joint"},
      {"leg_l6_joint", "l_leg_ankle_roll_joint"},
      {"leg_r6_joint", "r_leg_ankle_roll_joint"},
      {"arm_l1_joint", "l_arm_shoulder_pitch_joint"},
      {"arm_r1_joint", "r_arm_shoulder_pitch_joint"},
      {"arm_l2_joint", "l_arm_shoulder_roll_joint"},
      {"arm_r2_joint", "r_arm_shoulder_roll_joint"},
      {"arm_l3_joint", "l_arm_shoulder_yaw_joint"},
      {"arm_r3_joint", "r_arm_shoulder_yaw_joint"},
      {"arm_l4_joint", "l_arm_elbow_joint"},
      {"arm_r4_joint", "r_arm_elbow_joint"},
      {"l_arm_elbow_pitch_joint", "l_arm_elbow_joint"},
      {"r_arm_elbow_pitch_joint", "r_arm_elbow_joint"},
  };
  const auto it = aliases.find(name);
  return it == aliases.end() ? name : it->second;
}

}  // namespace legged
