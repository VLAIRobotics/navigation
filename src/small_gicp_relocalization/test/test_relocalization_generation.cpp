#include <gtest/gtest.h>

#include <small_gicp_relocalization/relocalization_generation.hpp>

TEST(RelocalizationGeneration, AcceptsCurrentResult)
{
  EXPECT_TRUE(small_gicp_relocalization::resultBelongsToInitialization(7, 7));
}

TEST(RelocalizationGeneration, RejectsResultFromPreviousInitialPose)
{
  EXPECT_FALSE(small_gicp_relocalization::resultBelongsToInitialization(7, 8));
}

TEST(RelocalizationGeneration, RejectsCurrentGenerationUntilInitialPoseIsApplied)
{
  EXPECT_FALSE(small_gicp_relocalization::resultBelongsToAppliedInitialization(8, 8, 0));
  EXPECT_FALSE(small_gicp_relocalization::resultBelongsToAppliedInitialization(7, 8, 7));
}

TEST(RelocalizationGeneration, AcceptsOnlyTheAppliedCurrentGeneration)
{
  EXPECT_TRUE(small_gicp_relocalization::resultBelongsToAppliedInitialization(8, 8, 8));
  EXPECT_FALSE(small_gicp_relocalization::resultBelongsToAppliedInitialization(0, 0, 0));
}
