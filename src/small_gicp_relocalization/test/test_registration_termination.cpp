#include <gtest/gtest.h>

#include "small_gicp_relocalization/registration_termination.hpp"

using small_gicp_relocalization::registrationTerminationAccepted;

TEST(RegistrationTermination, AcceptsReportedConvergenceBeforeTheIterationLimit)
{
  EXPECT_TRUE(registrationTerminationAccepted(true, 12U, 60U));
}

TEST(RegistrationTermination, AllowsIterationCappedCandidateForOtherQualityGates)
{
  EXPECT_TRUE(registrationTerminationAccepted(false, 60U, 60U));
}

TEST(RegistrationTermination, RejectsUnconvergedResultBeforeTheIterationLimit)
{
  EXPECT_FALSE(registrationTerminationAccepted(false, 59U, 60U));
}

TEST(RegistrationTermination, RejectsUnconvergedResultWhenLimitIsDisabled)
{
  EXPECT_FALSE(registrationTerminationAccepted(false, 0U, 0U));
}
