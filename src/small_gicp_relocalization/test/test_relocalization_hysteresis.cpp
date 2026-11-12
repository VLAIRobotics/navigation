#include <gtest/gtest.h>

#include <small_gicp_relocalization/relocalization_hysteresis.hpp>

TEST(RelocalizationHysteresis, KeepsLocalizedForTransientFailures)
{
  EXPECT_TRUE(small_gicp_relocalization::localizedFailureKeepsValidity(1, 3, 0.9, 1.0));
  EXPECT_TRUE(small_gicp_relocalization::localizedFailureKeepsValidity(2, 3, 1.0, 1.0));
}

TEST(RelocalizationHysteresis, LosesAfterFailureCountOrTimeout)
{
  EXPECT_FALSE(small_gicp_relocalization::localizedFailureKeepsValidity(3, 3, 0.1, 1.0));
  EXPECT_FALSE(small_gicp_relocalization::localizedFailureKeepsValidity(1, 3, 1.01, 1.0));
}
