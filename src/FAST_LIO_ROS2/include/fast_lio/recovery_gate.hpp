#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>

namespace fast_lio
{

class RecoveryGate
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Result
  {
    WAITING,
    READY
  };

  RecoveryGate(
    std::size_t required_frames = 5,
    std::chrono::duration<double> required_duration = std::chrono::duration<double>(0.5),
    double max_translation_error_m = 0.10,
    double max_rotation_error_rad = 0.05)
  : required_frames_(required_frames > 0 ? required_frames : 1),
    required_duration_(required_duration),
    max_translation_error_m_(max_translation_error_m),
    max_rotation_error_rad_(max_rotation_error_rad)
  {
  }

  void start()
  {
    active_ = true;
    consecutive_frames_ = 0;
    has_sequence_start_ = false;
    has_last_source_stamp_ = false;
  }

  void reset()
  {
    active_ = false;
    consecutive_frames_ = 0;
    has_sequence_start_ = false;
    has_last_source_stamp_ = false;
  }

  Result observe(
    bool candidate_valid, double translation_error_m, double rotation_error_rad,
    Clock::time_point now = Clock::now())
  {
    if (!active_) {
      return Result::READY;
    }
    if (!candidate_valid) {
      reset_sequence();
      return Result::WAITING;
    }
    if (!std::isfinite(translation_error_m) || !std::isfinite(rotation_error_rad) ||
      translation_error_m > max_translation_error_m_ ||
      rotation_error_rad > max_rotation_error_rad_)
    {
      reset_sequence();
      return Result::WAITING;
    }
    if (!has_sequence_start_) {
      sequence_start_ = now;
      has_sequence_start_ = true;
    }
    ++consecutive_frames_;
    if (consecutive_frames_ >= required_frames_ &&
      std::chrono::duration<double>(now - sequence_start_) >= required_duration_)
    {
      active_ = false;
      return Result::READY;
    }
    return Result::WAITING;
  }

  /*** Accumulate a candidate against per-scan motion-prediction error. A single
       large correction breaks this stability window; only ScanHealthMonitor's
       independent frame-and-time confirmation may request relocalization. ***/
  Result observe_candidate(
    bool candidate_valid, double translation_prediction_error_m,
    double rotation_prediction_error_rad, Clock::time_point now = Clock::now())
  {
    if (!active_) {
      return Result::READY;
    }
    if (!candidate_valid || !std::isfinite(translation_prediction_error_m) ||
      !std::isfinite(rotation_prediction_error_rad) ||
      translation_prediction_error_m > max_translation_error_m_ ||
      rotation_prediction_error_rad > max_rotation_error_rad_)
    {
      reset_sequence();
      return Result::WAITING;
    }
    if (!has_sequence_start_) {
      sequence_start_ = now;
      has_sequence_start_ = true;
    }
    ++consecutive_frames_;
    if (consecutive_frames_ >= required_frames_ &&
      std::chrono::duration<double>(now - sequence_start_) >= required_duration_)
    {
      active_ = false;
      return Result::READY;
    }
    return Result::WAITING;
  }

  /*** Production recovery uses sensor time so delayed processing cannot satisfy
       the 0.5 second stability requirement without 0.5 seconds of new scans. ***/
  Result observe_candidate(
    bool candidate_valid, double translation_prediction_error_m,
    double rotation_prediction_error_rad, double source_stamp)
  {
    if (!active_) {
      return Result::READY;
    }
    if (!std::isfinite(source_stamp) ||
      (has_last_source_stamp_ && source_stamp <= last_source_stamp_))
    {
      reset_sequence();
      return Result::WAITING;
    }
    last_source_stamp_ = source_stamp;
    has_last_source_stamp_ = true;
    if (!candidate_valid || !std::isfinite(translation_prediction_error_m) ||
      !std::isfinite(rotation_prediction_error_rad) ||
      translation_prediction_error_m > max_translation_error_m_ ||
      rotation_prediction_error_rad > max_rotation_error_rad_)
    {
      consecutive_frames_ = 0;
      has_sequence_start_ = false;
      return Result::WAITING;
    }
    if (!has_sequence_start_) {
      source_sequence_start_stamp_ = source_stamp;
      has_sequence_start_ = true;
    }
    ++consecutive_frames_;
    if (consecutive_frames_ >= required_frames_ &&
      source_stamp - source_sequence_start_stamp_ >= required_duration_.count())
    {
      active_ = false;
      return Result::READY;
    }
    return Result::WAITING;
  }

  std::size_t consecutive_frames() const { return consecutive_frames_; }
  bool active() const { return active_; }

private:
  void reset_sequence()
  {
    consecutive_frames_ = 0;
    has_sequence_start_ = false;
    has_last_source_stamp_ = false;
  }

  std::size_t required_frames_;
  std::chrono::duration<double> required_duration_;
  double max_translation_error_m_;
  double max_rotation_error_rad_;
  bool active_{false};
  bool has_sequence_start_{false};
  bool has_last_source_stamp_{false};
  std::size_t consecutive_frames_{0};
  Clock::time_point sequence_start_{};
  double last_source_stamp_{0.0};
  double source_sequence_start_stamp_{0.0};
};

}  // namespace fast_lio
