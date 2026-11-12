#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace bringup
{

enum class LifecycleReconcileAction : std::uint8_t
{
  NONE,
  STARTUP,
  PAUSE,
  RESUME,
};

struct LifecycleStateSummary
{
  bool known{false};
  bool all_active{false};
  bool all_inactive{false};
  bool any_unconfigured{false};
};

inline LifecycleStateSummary summarizeLifecycleStates(
  const std::vector<std::uint8_t> & state_ids, std::uint8_t unconfigured_state,
  std::uint8_t inactive_state, std::uint8_t active_state)
{
  LifecycleStateSummary summary;
  if (state_ids.empty()) {
    return summary;
  }
  summary.known = true;
  summary.all_active = true;
  summary.all_inactive = true;
  for (const auto state_id : state_ids) {
    if (state_id == active_state) {
      summary.all_inactive = false;
      continue;
    }
    summary.all_active = false;
    if (state_id == unconfigured_state) {
      summary.any_unconfigured = true;
    } else if (state_id != inactive_state) {
      summary.known = false;
      summary.all_inactive = false;
    }
  }
  return summary;
}

inline LifecycleReconcileAction reconcileLifecycle(
  bool desired_active, bool navigation_started, bool navigation_paused, bool command_pending)
{
  if (command_pending) {
    return LifecycleReconcileAction::NONE;
  }
  if (desired_active && !navigation_started) {
    return LifecycleReconcileAction::STARTUP;
  }
  if (desired_active && navigation_paused) {
    return LifecycleReconcileAction::RESUME;
  }
  if (!desired_active && navigation_started && !navigation_paused) {
    return LifecycleReconcileAction::PAUSE;
  }
  return LifecycleReconcileAction::NONE;
}

inline bool navigationGateShouldBeActive(
  bool localization_ready, bool require_fastlio_health, bool health_received,
  bool fastlio_healthy, double health_age_sec, double health_timeout_sec)
{
  if (!localization_ready) return false;
  if (!require_fastlio_health) return true;
  return health_received && fastlio_healthy && std::isfinite(health_age_sec) &&
         health_age_sec >= 0.0 && std::isfinite(health_timeout_sec) &&
         health_timeout_sec > 0.0 && health_age_sec <= health_timeout_sec;
}

inline bool updateLocalizationBaselineFromHandoff(
  bool baseline_ready, bool message_ready, bool handoff_shares_fastlio_health)
{
  return handoff_shares_fastlio_health ? (baseline_ready || message_ready) : message_ready;
}

inline bool navigationGateKeepsLifecycleDuringFastlioGrace(
  bool desired_active, bool localization_baseline_ready, bool health_loss_pending,
  bool terrain_ready)
{
  return desired_active && localization_baseline_ready && health_loss_pending && terrain_ready;
}

inline bool navigationGateTerrainStatusIsFresh(bool received, double age_sec, double timeout_sec)
{
  return received && std::isfinite(age_sec) && std::isfinite(timeout_sec) &&
         timeout_sec > 0.0 && age_sec >= 0.0 && age_sec <= timeout_sec;
}

inline bool navigationGateTerrainIsReady(
  bool required, bool status_valid, double age_sec, double timeout_sec)
{
  if (!required) return true;
  return status_valid && navigationGateTerrainStatusIsFresh(true, age_sec, timeout_sec);
}

inline bool navigationGateShouldStartTerrainRecoveryProbe(
  bool required, bool terrain_ready, bool navigation_ever_started,
  bool lifecycle_not_active, bool status_expired, bool probe_active, bool retry_ready)
{
  return required && !terrain_ready && navigation_ever_started && lifecycle_not_active &&
         status_expired && !probe_active && retry_ready;
}

inline bool navigationGateTerrainAllowsLifecycle(
  bool required, bool terrain_ready, bool initial_startup, bool recovery_probe_active)
{
  return !required || terrain_ready || initial_startup || recovery_probe_active;
}

inline bool navigationIsReady(bool desired_active, bool navigation_started, bool navigation_paused)
{
  return desired_active && navigation_started && !navigation_paused;
}

inline bool navigationIsActuallyReady(
  bool desired_active, bool manager_available, bool state_known, bool all_nodes_active,
  bool command_pending)
{
  return desired_active && manager_available && state_known && all_nodes_active && !command_pending;
}

inline bool lifecycleRequestExpired(bool command_pending, double elapsed_sec, double timeout_sec)
{
  return command_pending && std::isfinite(elapsed_sec) && std::isfinite(timeout_sec) &&
    timeout_sec > 0.0 && elapsed_sec >= timeout_sec;
}

}  // namespace bringup
