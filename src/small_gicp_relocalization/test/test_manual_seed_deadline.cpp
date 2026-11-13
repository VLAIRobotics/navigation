#include <gtest/gtest.h>

#include <small_gicp_relocalization/manual_seed_deadline.hpp>

using small_gicp_relocalization::ManualSeedDeadlineState;

TEST(ManualSeedDeadline, WaitsForInputFromPoseReceipt)
{
  EXPECT_EQ(small_gicp_relocalization::manualSeedDeadlineState(
    true, false, 2.9, 0.0, 3.0, 10.0), ManualSeedDeadlineState::WAITING_FOR_INPUT);
  EXPECT_EQ(small_gicp_relocalization::manualSeedDeadlineState(
    true, false, 3.0, 0.0, 3.0, 10.0), ManualSeedDeadlineState::EXPIRED);
}

TEST(ManualSeedDeadline, ValidationDeadlineStartsAtUsableCloud)
{
  EXPECT_EQ(small_gicp_relocalization::manualSeedDeadlineState(
    true, true, 20.0, 9.9, 3.0, 10.0), ManualSeedDeadlineState::VALIDATING);
  EXPECT_EQ(small_gicp_relocalization::manualSeedDeadlineState(
    true, true, 20.0, 10.0, 3.0, 10.0), ManualSeedDeadlineState::EXPIRED);
}

TEST(ManualSeedDeadline, InactiveManualSeedDoesNotExpire)
{
  EXPECT_EQ(small_gicp_relocalization::manualSeedDeadlineState(
    false, false, 100.0, 100.0, 3.0, 10.0), ManualSeedDeadlineState::INACTIVE);
}
