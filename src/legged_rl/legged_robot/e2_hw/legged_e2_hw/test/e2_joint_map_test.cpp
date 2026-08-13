#include "E2JointMap.h"

#include <gtest/gtest.h>

#include <array>

namespace legged
{
TEST(E2JointMapTest, CoversEveryLogicalJointExactlyOnce)
{
  std::array<int, 23> counts{};
  for (const auto& channel : kE2JointChannels) {
    ASSERT_LT(channel.jointIndex, counts.size());
    ++counts[channel.jointIndex];
  }
  for (const int count : counts) EXPECT_EQ(count, 1);
}

TEST(E2JointMapTest, CoversExpectedControlImageWithoutDuplicates)
{
  std::array<std::array<int, 6>, 5> counts{};
  const std::array<std::size_t, 5> expectedCounts{{4, 4, 6, 6, 3}};
  for (const auto& channel : kE2JointChannels) {
    ASSERT_LT(channel.controlGroup, counts.size());
    ASSERT_LT(channel.groupIndex, expectedCounts[channel.controlGroup]);
    ++counts[channel.controlGroup][channel.groupIndex];
  }
  for (std::size_t group = 0; group < expectedCounts.size(); ++group) {
    for (std::size_t index = 0; index < expectedCounts[group]; ++index)
      EXPECT_EQ(counts[group][index], 1);
  }
}

TEST(E2JointMapTest, OnlyParallelChannelsDisableDirectPd)
{
  std::array<std::size_t, 6> expected{{12, 13, 18, 19, 21, 22}};
  std::size_t found = 0;
  for (const auto& channel : kE2JointChannels) {
    if (!channel.directPd) {
      ASSERT_LT(found, expected.size());
      EXPECT_EQ(channel.jointIndex, expected[found++]);
    }
  }
  EXPECT_EQ(found, expected.size());
}
}  // namespace legged

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
