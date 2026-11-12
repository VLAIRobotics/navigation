#include <gtest/gtest.h>

#include <limits>

#include <small_gicp_relocalization/fastlio_health.hpp>

TEST(FastlioHealth, RequiresARecentHealthyMessage)
{
  EXPECT_TRUE(small_gicp_relocalization::fastlioHealthIsUsable(true, true, 0.49, 0.5));
  EXPECT_FALSE(small_gicp_relocalization::fastlioHealthIsUsable(false, true, 0.0, 0.5));
  EXPECT_FALSE(small_gicp_relocalization::fastlioHealthIsUsable(true, false, 0.0, 0.5));
}

TEST(FastlioHealth, RejectsExpiredOrInvalidAge)
{
  EXPECT_FALSE(small_gicp_relocalization::fastlioHealthIsUsable(true, true, 0.51, 0.5));
  EXPECT_FALSE(small_gicp_relocalization::fastlioHealthIsUsable(true, true, -0.1, 0.5));
  EXPECT_FALSE(small_gicp_relocalization::fastlioHealthIsUsable(
    true, true, std::numeric_limits<double>::quiet_NaN(), 0.5));
}
