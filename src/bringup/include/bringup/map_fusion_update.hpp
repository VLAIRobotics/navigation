#ifndef BRINGUP__MAP_FUSION_UPDATE_HPP_
#define BRINGUP__MAP_FUSION_UPDATE_HPP_

#include <cstdint>
#include <optional>

namespace bringup
{

inline std::optional<std::int8_t> groundEvidenceUpdate(
  std::int8_t occupancy, bool same_scan_has_nonground,
  std::uint8_t consecutive_ground_frames,
  int free_observations, int clear_observations)
{
  if (same_scan_has_nonground) return std::nullopt;
  if (occupancy < 0 && consecutive_ground_frames >= free_observations) {
    return static_cast<std::int8_t>(0);
  }
  if (occupancy >= 65 && consecutive_ground_frames >= clear_observations) {
    return static_cast<std::int8_t>(0);
  }
  return std::nullopt;
}

inline std::optional<std::int8_t> onlineObstacleRayClearUpdate(
  std::int8_t occupancy, bool prior_occupied, bool online_occupied,
  std::uint8_t consecutive_free_ray_frames, int clear_observations)
{
  if (!prior_occupied && online_occupied && occupancy >= 65 &&
    consecutive_free_ray_frames >= clear_observations)
  {
    return static_cast<std::int8_t>(0);
  }
  return std::nullopt;
}

inline std::optional<std::int8_t> onlineObstacleClearEventUpdate(
  std::int8_t occupancy, bool prior_occupied, bool online_occupied)
{
  if (!prior_occupied && online_occupied && occupancy >= 65) {
    return static_cast<std::int8_t>(0);
  }
  return std::nullopt;
}

inline bool obstacleHitVetoesClearEvent(
  std::int64_t clear_event_stamp_ns, std::int64_t latest_obstacle_hit_stamp_ns)
{
  return clear_event_stamp_ns <= 0 || latest_obstacle_hit_stamp_ns >= clear_event_stamp_ns;
}

inline bool obstacleHitVetoesRayClear(
  std::int64_t ray_stamp_ns, std::int64_t latest_obstacle_hit_stamp_ns)
{
  return ray_stamp_ns <= 0 || latest_obstacle_hit_stamp_ns >= ray_stamp_ns;
}

inline bool obstacleEvidenceFrameReady(
  std::int64_t clear_event_stamp_ns, bool same_frame_obstacle_evidence_processed,
  std::int64_t confirmed_published_cutoff_ns, std::int64_t obstacle_evidence_watermark_ns)
{
  // The caller also checks the publisher ID and every sequence through the fence. These
  // timestamp checks keep the event and the confirmed publication boundary aligned.
  return clear_event_stamp_ns > 0 && same_frame_obstacle_evidence_processed &&
         confirmed_published_cutoff_ns >= clear_event_stamp_ns &&
         obstacle_evidence_watermark_ns >= confirmed_published_cutoff_ns;
}

inline bool rayClearEvidenceReady(
  std::int64_t ray_frame_stamp_ns, bool same_frame_obstacle_evidence_processed)
{
  return ray_frame_stamp_ns > 0 && same_frame_obstacle_evidence_processed;
}

inline bool globalClearEvidenceReady(
  std::int64_t clear_event_stamp_ns, bool same_frame_obstacle_evidence_processed,
  std::int64_t confirmed_published_cutoff_ns, std::int64_t obstacle_evidence_watermark_ns,
  bool pending_blocker_at_or_after_event, std::int64_t incomplete_blocker_through_stamp_ns)
{
  return !pending_blocker_at_or_after_event &&
         incomplete_blocker_through_stamp_ns < clear_event_stamp_ns &&
         obstacleEvidenceFrameReady(
           clear_event_stamp_ns, same_frame_obstacle_evidence_processed,
           confirmed_published_cutoff_ns,
           obstacle_evidence_watermark_ns);
}

}  // namespace bringup

#endif  // BRINGUP__MAP_FUSION_UPDATE_HPP_
