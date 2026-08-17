#include "rl_controllers/PdTrajectoryLoader.h"
#include "rl_controllers/JointNameAliases.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace
{
std::string writeTempCsv(const std::string& contents)
{
  const std::string path =
      "/tmp/pd_traj_test_" + std::to_string(getpid()) + "_" + std::to_string(rand()) + ".csv";
  std::ofstream out(path.c_str());
  EXPECT_TRUE(out.is_open());
  out << contents;
  return path;
}
}  // namespace

TEST(JointNameAliasesTest, MapsE1NamesToE2)
{
  EXPECT_EQ(legged::normalizeControlJointName("arm_l1_joint"), "l_arm_shoulder_pitch_joint");
  EXPECT_EQ(legged::normalizeControlJointName("arm_l4_joint"), "l_arm_elbow_joint");
  EXPECT_EQ(legged::normalizeControlJointName("leg_l3_joint"), "l_leg_hip_pitch_joint");
  EXPECT_EQ(legged::normalizeControlJointName("waist_1_joint"), "waist_yaw_joint");
  EXPECT_EQ(legged::normalizeControlJointName("l_arm_elbow_joint"), "l_arm_elbow_joint");
}

TEST(PdTrajectoryLoaderTest, LoadsSparseArmCsvAndHoldsUnmappedJoints)
{
  const std::string csv =
      "time,l_arm_shoulder_pitch_joint,l_arm_elbow_joint\n"
      "0.000,0.10,-0.20\n"
      "0.002,0.12,-0.22\n"
      "0.004,0.14,-0.24\n";
  const std::string path = writeTempCsv(csv);
  const std::vector<std::string> joints{
      "l_leg_hip_pitch_joint",
      "l_arm_shoulder_pitch_joint",
      "l_arm_elbow_joint",
      "r_arm_elbow_joint",
  };

  legged::PdTrajectoryData data;
  std::string error;
  ASSERT_TRUE(legged::loadMultiJointTrajectoryCsv(path, joints, false, data, &error)) << error;
  EXPECT_EQ(data.samples.size(), 3u);
  EXPECT_NEAR(data.sample_interval, 0.002, 1e-9);
  ASSERT_EQ(data.joint_from_csv.size(), joints.size());
  EXPECT_FALSE(data.joint_from_csv[0]);
  EXPECT_TRUE(data.joint_from_csv[1]);
  EXPECT_TRUE(data.joint_from_csv[2]);
  EXPECT_FALSE(data.joint_from_csv[3]);
  EXPECT_NEAR(data.samples[0][1], 0.10, 1e-9);
  EXPECT_NEAR(data.samples[2][2], -0.24, 1e-9);
  EXPECT_NEAR(data.samples[1][0], 0.0, 1e-9);
  std::remove(path.c_str());
}

TEST(PdTrajectoryLoaderTest, AcceptsE1AliasHeaders)
{
  const std::string csv =
      "time,arm_l1_joint,arm_r4_joint\n"
      "0.0,0.3,-0.5\n"
      "0.002,0.4,-0.6\n";
  const std::string path = writeTempCsv(csv);
  const std::vector<std::string> joints{
      "l_arm_shoulder_pitch_joint",
      "r_arm_elbow_joint",
  };

  legged::PdTrajectoryData data;
  ASSERT_TRUE(legged::loadMultiJointTrajectoryCsv(path, joints, false, data, nullptr));
  EXPECT_TRUE(data.joint_from_csv[0]);
  EXPECT_TRUE(data.joint_from_csv[1]);
  EXPECT_NEAR(data.samples[1][0], 0.4, 1e-9);
  EXPECT_NEAR(data.samples[1][1], -0.6, 1e-9);
  std::remove(path.c_str());
}

TEST(PdTrajectoryLoaderTest, RejectsMissingTimeColumn)
{
  const std::string csv = "l_arm_elbow_joint\n0.1\n";
  const std::string path = writeTempCsv(csv);
  const std::vector<std::string> joints{"l_arm_elbow_joint"};
  legged::PdTrajectoryData data;
  std::string error;
  EXPECT_FALSE(legged::loadMultiJointTrajectoryCsv(path, joints, false, data, &error));
  EXPECT_NE(error.find("time"), std::string::npos);
  std::remove(path.c_str());
}

#ifdef TEST_ARM_WAVE_CSV
TEST(PdTrajectoryLoaderTest, LoadsPackagedE2ArmWave)
{
  const std::vector<std::string> joints{
      "l_leg_hip_pitch_joint", "l_leg_hip_roll_joint", "l_leg_hip_yaw_joint",
      "l_leg_knee_joint", "l_leg_ankle_pitch_joint", "l_leg_ankle_roll_joint",
      "r_leg_hip_pitch_joint", "r_leg_hip_roll_joint", "r_leg_hip_yaw_joint",
      "r_leg_knee_joint", "r_leg_ankle_pitch_joint", "r_leg_ankle_roll_joint",
      "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
      "l_arm_shoulder_pitch_joint", "l_arm_shoulder_roll_joint",
      "l_arm_shoulder_yaw_joint", "l_arm_elbow_joint",
      "r_arm_shoulder_pitch_joint", "r_arm_shoulder_roll_joint",
      "r_arm_shoulder_yaw_joint", "r_arm_elbow_joint",
  };
  legged::PdTrajectoryData data;
  std::string error;
  ASSERT_TRUE(legged::loadMultiJointTrajectoryCsv(TEST_ARM_WAVE_CSV, joints, false, data, &error))
      << error;
  EXPECT_EQ(data.samples.size(), 2001u);
  EXPECT_NEAR(data.sample_interval, 0.002, 1e-6);
  int mapped = 0;
  for (bool from_csv : data.joint_from_csv)
  {
    if (from_csv)
    {
      ++mapped;
    }
  }
  EXPECT_EQ(mapped, 8);
  EXPECT_NEAR(data.samples.front()[15], 0.0, 1e-6);   // l_arm_shoulder_pitch
  EXPECT_NEAR(data.samples.front()[16], 0.2618, 1e-4);  // l_arm_shoulder_roll
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
