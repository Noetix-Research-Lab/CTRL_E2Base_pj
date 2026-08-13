#include "PvtCommandSafety.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{

void ExpectLogicalZero(const PvtCommandValues& command)
{
  EXPECT_FLOAT_EQ(command.kp, 0.0F);
  EXPECT_FLOAT_EQ(command.kd, 0.0F);
  EXPECT_FLOAT_EQ(command.pos, 0.0F);
  EXPECT_FLOAT_EQ(command.spd, 0.0F);
  EXPECT_FLOAT_EQ(command.tor, 0.0F);
}

TEST(PvtCommandSafetyTest, AcceptsFiniteCommand)
{
  PvtCommandValues command{10.0F, 1.0F, 0.5F, -0.25F, 2.0F};
  EXPECT_TRUE(SanitizePvtCommandValues(command));
  EXPECT_FLOAT_EQ(command.kp, 10.0F);
  EXPECT_FLOAT_EQ(command.tor, 2.0F);
}

TEST(PvtCommandSafetyTest, RejectsAndClearsEveryNonFiniteField)
{
  const float invalidValues[] = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity()};

  for (float invalid : invalidValues) {
    for (int field = 0; field < 5; ++field) {
      PvtCommandValues command{1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
      float* fields[] = {&command.kp, &command.kd, &command.pos,
                         &command.spd, &command.tor};
      *fields[field] = invalid;
      EXPECT_FALSE(SanitizePvtCommandValues(command));
      ExpectLogicalZero(command);
    }
  }
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
