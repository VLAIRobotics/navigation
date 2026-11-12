#include <cmath>
#include <chrono>

#include <gtest/gtest.h>

#include "bringup/navigation_gate_state.hpp"
#include "bringup/status_heartbeat.hpp"

TEST(NavigationGateState, ReconcilesResumeAfterLocalizationRecovers)
{
  EXPECT_EQ(
    bringup::reconcileLifecycle(true, true, true, false),
    bringup::LifecycleReconcileAction::RESUME);
}

TEST(NavigationGateState, ReconcilesPauseWhenLocalizationIsLost)
{
  EXPECT_EQ(
    bringup::reconcileLifecycle(false, true, false, false),
    bringup::LifecycleReconcileAction::PAUSE);
}

TEST(NavigationGateState, DefersWhileAnotherLifecycleCommandIsPending)
{
  EXPECT_EQ(
    bringup::reconcileLifecycle(false, true, false, true),
    bringup::LifecycleReconcileAction::NONE);
}

TEST(NavigationGateState, StartsOnlyWhenDesired)
{
  EXPECT_EQ(
    bringup::reconcileLifecycle(true, false, false, false),
    bringup::LifecycleReconcileAction::STARTUP);
  EXPECT_EQ(
    bringup::reconcileLifecycle(false, false, false, false),
    bringup::LifecycleReconcileAction::NONE);
}

TEST(NavigationGateState, DoesNotReportReadyWithoutActualActiveState)
{
  EXPECT_FALSE(bringup::navigationIsActuallyReady(true, false, true, true, false));
  EXPECT_FALSE(bringup::navigationIsActuallyReady(true, true, false, true, false));
  EXPECT_FALSE(bringup::navigationIsActuallyReady(true, true, true, false, false));
  EXPECT_FALSE(bringup::navigationIsActuallyReady(true, true, true, true, true));
  EXPECT_TRUE(bringup::navigationIsActuallyReady(true, true, true, true, false));
}

TEST(NavigationGateState, LifecycleRequestExpiresAtConfiguredDeadline)
{
  EXPECT_FALSE(bringup::lifecycleRequestExpired(false, 10.0, 2.0));
  EXPECT_FALSE(bringup::lifecycleRequestExpired(true, 1.9, 2.0));
  EXPECT_TRUE(bringup::lifecycleRequestExpired(true, 2.0, 2.0));
  EXPECT_FALSE(bringup::lifecycleRequestExpired(true, 1.0, 0.0));
}

TEST(NavigationGateState, SummarizesActualLifecycleStates)
{
  const auto active = bringup::summarizeLifecycleStates({3, 3}, 1, 2, 3);
  EXPECT_TRUE(active.known);
  EXPECT_TRUE(active.all_active);
  EXPECT_FALSE(active.any_unconfigured);

  const auto inactive = bringup::summarizeLifecycleStates({2, 3}, 1, 2, 3);
  EXPECT_TRUE(inactive.known);
  EXPECT_FALSE(inactive.all_active);
  EXPECT_FALSE(inactive.any_unconfigured);

  const auto unconfigured = bringup::summarizeLifecycleStates({1, 3}, 1, 2, 3);
  EXPECT_TRUE(unconfigured.known);
  EXPECT_FALSE(unconfigured.all_active);
  EXPECT_TRUE(unconfigured.any_unconfigured);

  EXPECT_FALSE(bringup::summarizeLifecycleStates({0}, 1, 2, 3).known);
  EXPECT_FALSE(bringup::summarizeLifecycleStates({}, 1, 2, 3).known);
}

TEST(NavigationGateState, SeparatesStatusHeartbeatFreshnessFromLocalizationValidity)
{
  EXPECT_TRUE(bringup::localizationStatusHeartbeatIsFresh(true, 0.9, 1.0));
  EXPECT_FALSE(bringup::localizationStatusHeartbeatIsFresh(true, 1.1, 1.0));
  EXPECT_FALSE(bringup::localizationStatusHeartbeatIsFresh(false, 0.0, 1.0));
}

TEST(NavigationGateState, ReadyHeartbeatStaysFreshAcrossMultipleSearchPeriods)
{
  constexpr double search_period_sec = 2.0;
  const double heartbeat_period_sec = std::chrono::duration<double>(
    bringup::kNavigationReadyHeartbeatPeriod).count();
  const auto heartbeat_count = static_cast<int>(
    std::ceil(2.0 * search_period_sec / heartbeat_period_sec));
  for (int i = 0; i < heartbeat_count; ++i) {
    EXPECT_TRUE(bringup::navigationReadyHeartbeatIsFresh(true, heartbeat_period_sec));
  }
  EXPECT_FALSE(bringup::navigationReadyHeartbeatIsFresh(false, 0.0));
  EXPECT_FALSE(bringup::navigationReadyHeartbeatIsFresh(
      true, bringup::kNavigationReadyHeartbeatTimeoutSec + heartbeat_period_sec));
}


TEST(NavigationGateState, RequiresFreshHealthyFastLioWhenConfigured)
{
  EXPECT_FALSE(bringup::navigationGateShouldBeActive(true, true, false, true, 0.0, 0.5));
  EXPECT_FALSE(bringup::navigationGateShouldBeActive(true, true, true, false, 0.1, 0.5));
  EXPECT_FALSE(bringup::navigationGateShouldBeActive(true, true, true, true, 0.6, 0.5));
  EXPECT_TRUE(bringup::navigationGateShouldBeActive(true, true, true, true, 0.5, 0.5));
  EXPECT_TRUE(bringup::navigationGateShouldBeActive(true, false, false, false, 10.0, 0.0));
  EXPECT_FALSE(bringup::navigationGateShouldBeActive(false, false, true, true, 0.0, 0.5));
}

TEST(NavigationGateState, HealthCallbackBeforeSharedHandoffFalseRetainsBaselineDuringGrace)
{
  bool baseline_ready = true;
  bool health_received = true;
  bool healthy = true;
  bool loss_pending = false;
  const auto health_callback = [&]() {
      baseline_ready = bringup::updateLocalizationBaselineFromHandoff(
        baseline_ready, false, true);
      healthy = false;
      loss_pending = true;
    };
  const auto handoff_callback = [&]() {
      baseline_ready = bringup::updateLocalizationBaselineFromHandoff(
        baseline_ready, false, true);
    };

  health_callback();
  handoff_callback();

  EXPECT_TRUE(baseline_ready);
  EXPECT_FALSE(bringup::navigationGateShouldBeActive(
      baseline_ready, true, health_received, healthy, 0.0, 0.5));
  EXPECT_TRUE(bringup::navigationGateKeepsLifecycleDuringFastlioGrace(
      true, baseline_ready, loss_pending, true));

  baseline_ready = bringup::updateLocalizationBaselineFromHandoff(
    baseline_ready, true, true);
  healthy = true;
  loss_pending = false;
  EXPECT_TRUE(bringup::navigationGateShouldBeActive(
      baseline_ready, true, health_received, healthy, 0.0, 0.5));
}

TEST(NavigationGateState, SharedHandoffFalseBeforeHealthCallbackRetainsBaselineDuringGrace)
{
  bool baseline_ready = true;
  bool health_received = true;
  bool healthy = true;
  bool loss_pending = false;
  const auto health_callback = [&]() {
      baseline_ready = bringup::updateLocalizationBaselineFromHandoff(
        baseline_ready, false, true);
      healthy = false;
      loss_pending = true;
    };
  const auto handoff_callback = [&]() {
      baseline_ready = bringup::updateLocalizationBaselineFromHandoff(
        baseline_ready, false, true);
    };

  handoff_callback();
  health_callback();

  EXPECT_TRUE(baseline_ready);
  EXPECT_FALSE(bringup::navigationGateShouldBeActive(
      baseline_ready, true, health_received, healthy, 0.0, 0.5));
  EXPECT_TRUE(bringup::navigationGateKeepsLifecycleDuringFastlioGrace(
      true, baseline_ready, loss_pending, true));

  baseline_ready = bringup::updateLocalizationBaselineFromHandoff(
    baseline_ready, true, true);
  healthy = true;
  loss_pending = false;
  EXPECT_TRUE(bringup::navigationGateShouldBeActive(
      baseline_ready, true, health_received, healthy, 0.0, 0.5));
}

TEST(NavigationGateState, RequiresFreshTerrainOnlyWhenConfigured)
{
  EXPECT_FALSE(bringup::navigationGateTerrainIsReady(true, false, 0.0, 1.2));
  EXPECT_FALSE(bringup::navigationGateTerrainIsReady(true, true, 1.3, 1.2));
  EXPECT_FALSE(bringup::navigationGateTerrainIsReady(true, true, -0.2, 1.2));
  EXPECT_TRUE(bringup::navigationGateTerrainIsReady(true, true, 1.2, 1.2));
  EXPECT_TRUE(bringup::navigationGateTerrainIsReady(false, false, 10.0, 0.0));
}

TEST(NavigationGateState, TerrainStatusHeartbeatExpiresIndependentlyOfValidity)
{
  EXPECT_TRUE(bringup::navigationGateTerrainStatusIsFresh(true, 1.2, 1.2));
  EXPECT_FALSE(bringup::navigationGateTerrainStatusIsFresh(false, 0.0, 1.2));
  EXPECT_FALSE(bringup::navigationGateTerrainStatusIsFresh(true, 1.3, 1.2));
}

TEST(NavigationGateState, TerrainGateAllowsInitialLifecycleForStatusBootstrap)
{
  EXPECT_TRUE(bringup::navigationGateTerrainAllowsLifecycle(true, false, true, false));
  EXPECT_FALSE(bringup::navigationGateTerrainAllowsLifecycle(true, false, false, false));
  EXPECT_TRUE(bringup::navigationGateTerrainAllowsLifecycle(true, true, false, false));
  EXPECT_TRUE(bringup::navigationGateTerrainAllowsLifecycle(false, false, false, false));
}

TEST(NavigationGateState, StartsTerrainRecoveryProbeOnlyAfterExpiredStatusAndInactiveLifecycle)
{
  EXPECT_FALSE(bringup::navigationGateShouldStartTerrainRecoveryProbe(
      true, false, false, true, true, false, true));
  EXPECT_FALSE(bringup::navigationGateShouldStartTerrainRecoveryProbe(
      true, false, true, false, true, false, true));
  EXPECT_FALSE(bringup::navigationGateShouldStartTerrainRecoveryProbe(
      true, false, true, true, false, false, true));
  EXPECT_FALSE(bringup::navigationGateShouldStartTerrainRecoveryProbe(
      true, false, true, true, true, true, true));
  EXPECT_FALSE(bringup::navigationGateShouldStartTerrainRecoveryProbe(
      true, false, true, true, true, false, false));
  EXPECT_TRUE(bringup::navigationGateShouldStartTerrainRecoveryProbe(
      true, false, true, true, true, false, true));
}
