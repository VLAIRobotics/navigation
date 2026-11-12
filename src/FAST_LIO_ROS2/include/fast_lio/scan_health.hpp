#pragma once

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>

namespace fast_lio
{

/*** Reasons a single scan can be judged abnormal. They are collected as a bitset
     for the whole frame, so one bad frame raises the consecutive-fault counter
     exactly once no matter how many gates it trips. ***/
enum ScanFaultReason : std::uint32_t
{
    SCAN_FAULT_NONE               = 0u,
    SCAN_FAULT_LIDAR_GAP          = 1u << 0,  // processed scan timestamps too far apart
    SCAN_FAULT_IMU_GAP            = 1u << 1,  // IMU samples missing
    SCAN_FAULT_EMPTY_SCAN         = 1u << 2,  // nothing usable left after undistortion
    SCAN_FAULT_MAP_NOT_READY      = 1u << 3,  // kdtree still being seeded
    SCAN_FAULT_MEASUREMENT_GATE   = 1u << 4,  // too few or too noisy correspondences
    SCAN_FAULT_IMPLAUSIBLE_UPDATE = 1u << 5,  // correction beyond the motion limits
    SCAN_FAULT_NON_FINITE_STATE   = 1u << 6,  // NaN/Inf in the state or covariance
    SCAN_FAULT_TIME_REGRESSION    = 1u << 7,  // sensor stamps went backwards
    SCAN_FAULT_LIDAR_QUEUE_DROP   = 1u << 8,  // unclaimed scans dropped under load
};

/*** Faults the filter cannot estimate its way out of: the state or the time axis
     itself is broken, so it has to be re-initialised instead of counted. ***/
constexpr std::uint32_t SCAN_FAULT_FATAL_MASK =
    SCAN_FAULT_NON_FINITE_STATE | SCAN_FAULT_TIME_REGRESSION;

inline bool lidar_gap_exceeds_threshold(double measured_gap, double threshold)
{
    return measured_gap > threshold;
}

enum class ScanOutcome : std::uint8_t
{
    ACCEPTED,  // trustworthy: publish everything and fold the scan into the map
    REJECTED,  // degraded: publish the propagated state only, never touch the map
  FATAL,     // corrupt state or time axis: isolate immediately
  RELOCALIZATION_REQUIRED,  // a continuous abnormal scan run met both confirmation limits
  DUPLICATE,  // duplicate or out-of-order source scan; discard without changing counters
};

struct ScanHealthDecision
{
    ScanOutcome outcome{ScanOutcome::ACCEPTED};
    std::uint32_t reasons{SCAN_FAULT_NONE};
    bool hard_fault{false};
    int consecutive_faults{0};
    double fault_duration_sec{0.0};

    bool healthy() const { return outcome == ScanOutcome::ACCEPTED; }
    bool may_update_map() const { return outcome == ScanOutcome::ACCEPTED; }
};

inline std::string scan_fault_reason_text(std::uint32_t reasons)
{
    struct Entry
    {
        std::uint32_t bit;
        const char *text;
    };
    static const Entry entries[] = {
        {SCAN_FAULT_LIDAR_GAP, "lidar gap"},
        {SCAN_FAULT_IMU_GAP, "imu gap"},
        {SCAN_FAULT_EMPTY_SCAN, "empty scan"},
        {SCAN_FAULT_MAP_NOT_READY, "map not ready"},
        {SCAN_FAULT_MEASUREMENT_GATE, "measurement gate"},
        {SCAN_FAULT_IMPLAUSIBLE_UPDATE, "implausible update"},
        {SCAN_FAULT_NON_FINITE_STATE, "non-finite state"},
        {SCAN_FAULT_TIME_REGRESSION, "time regression"},
        {SCAN_FAULT_LIDAR_QUEUE_DROP, "lidar queue drop"},
    };

    std::string text;
    for (const Entry &entry : entries)
    {
        if ((reasons & entry.bit) == 0u) continue;
        if (!text.empty()) text += ", ";
        text += entry.text;
    }
    return text.empty() ? std::string("none") : text;
}

/*** Per-frame health state machine. It is owned by the processing thread; sensor
     callbacks only raise reason flags for it to consume. ***/
class ScanHealthMonitor
{
public:
    explicit ScanHealthMonitor(
        int max_consecutive_faults = 10,
        double min_fault_duration_sec = 1.0,
        double max_inter_scan_gap_sec = 0.5)
    {
        set_max_consecutive_faults(max_consecutive_faults);
        set_min_fault_duration(min_fault_duration_sec);
        set_max_inter_scan_gap(max_inter_scan_gap_sec);
    }

    void set_max_consecutive_faults(int max_consecutive_faults)
    {
        max_consecutive_faults_ = max_consecutive_faults > 0 ? max_consecutive_faults : 1;
    }

    void set_min_fault_duration(double min_fault_duration_sec)
    {
        min_fault_duration_sec_ = std::isfinite(min_fault_duration_sec) &&
            min_fault_duration_sec > 0.0 ? min_fault_duration_sec : 0.0;
    }

    void set_max_inter_scan_gap(double max_inter_scan_gap_sec)
    {
        max_inter_scan_gap_sec_ = std::isfinite(max_inter_scan_gap_sec) &&
            max_inter_scan_gap_sec > 0.0 ? max_inter_scan_gap_sec : 0.0;
    }

    /*** Evaluate one source-timestamped scan. Only a strictly newer scan can
         advance or clear the abnormal run; wall timers and duplicate messages
         cannot manufacture evidence. ***/
    ScanHealthDecision evaluate(std::uint32_t reasons, double source_stamp)
    {
        ScanHealthDecision decision;
        decision.reasons = reasons;

        const std::uint32_t health_reasons = reasons & ~SCAN_FAULT_LIDAR_QUEUE_DROP;
        if ((health_reasons & SCAN_FAULT_FATAL_MASK) != 0u || !std::isfinite(source_stamp))
        {
            hard_fault_ = true;
            decision.outcome = ScanOutcome::FATAL;
            decision.hard_fault = true;
            decision.consecutive_faults = consecutive_faults_;
            return decision;
        }

        if (have_last_source_stamp_ && source_stamp <= last_source_stamp_)
        {
            decision.outcome = ScanOutcome::DUPLICATE;
            decision.hard_fault = hard_fault_;
            decision.consecutive_faults = consecutive_faults_;
            decision.fault_duration_sec = fault_duration_sec_;
            return decision;
        }

        if (have_last_source_stamp_ && max_inter_scan_gap_sec_ > 0.0 &&
            source_stamp - last_source_stamp_ >= max_inter_scan_gap_sec_)
        {
            reset_fault_run();
        }

        last_source_stamp_ = source_stamp;
        have_last_source_stamp_ = true;

        if (health_reasons == SCAN_FAULT_NONE)
        {
            reset_fault_run();
            decision.outcome = ScanOutcome::ACCEPTED;
            decision.hard_fault = hard_fault_;
            return decision;
        }

        if (consecutive_faults_ == 0)
        {
            fault_run_start_stamp_ = source_stamp;
        }
        ++consecutive_faults_;
        fault_duration_sec_ = std::max(0.0, source_stamp - fault_run_start_stamp_);
        decision.consecutive_faults = consecutive_faults_;
        decision.fault_duration_sec = fault_duration_sec_;

        if (consecutive_faults_ >= max_consecutive_faults_ &&
            fault_duration_sec_ >= min_fault_duration_sec_)
        {
            hard_fault_ = true;
            decision.outcome = ScanOutcome::RELOCALIZATION_REQUIRED;
        }
        else
        {
            decision.outcome = ScanOutcome::REJECTED;
        }
        decision.hard_fault = hard_fault_;
        return decision;
    }

    int consecutive_faults() const { return consecutive_faults_; }
    int max_consecutive_faults() const { return max_consecutive_faults_; }
    bool hard_fault() const { return hard_fault_; }
    double min_fault_duration() const { return min_fault_duration_sec_; }
    double max_inter_scan_gap() const { return max_inter_scan_gap_sec_; }

    void reset()
    {
        consecutive_faults_ = 0;
        hard_fault_ = false;
        fault_run_start_stamp_ = 0.0;
        fault_duration_sec_ = 0.0;
        last_source_stamp_ = 0.0;
        have_last_source_stamp_ = false;
    }

    void reset_fault_run()
    {
        consecutive_faults_ = 0;
        fault_run_start_stamp_ = 0.0;
        fault_duration_sec_ = 0.0;
    }

private:
    int max_consecutive_faults_{10};
    double min_fault_duration_sec_{1.0};
    double max_inter_scan_gap_sec_{0.5};
    int consecutive_faults_{0};
    bool hard_fault_{false};
    double fault_run_start_stamp_{0.0};
    double fault_duration_sec_{0.0};
    double last_source_stamp_{0.0};
    bool have_last_source_stamp_{false};
};

template <typename CloudPtr>
struct LidarScan
{
    CloudPtr cloud{};
    double timestamp{0.0};
    std::chrono::steady_clock::time_point received_at{};
    std::size_t raw_point_count{0};
};

/*** Bounded queue of LiDAR scans waiting for IMU. The scan handed to the filter
     is moved into a pending slot first, so an overflowing callback can only drop
     scans that nobody has claimed yet. ***/
template <typename CloudPtr>
class LidarScanQueue
{
public:
    explicit LidarScanQueue(std::size_t capacity = 5)
    : capacity_(capacity > 0 ? capacity : 1)
    {
    }

    /*** Returns how many unclaimed scans had to be dropped to stay in capacity. ***/
    std::size_t push(LidarScan<CloudPtr> scan)
    {
        queue_.push_back(std::move(scan));
        std::size_t dropped = 0;
        while (queue_.size() > capacity_)
        {
            queue_.pop_front();
            ++dropped;
        }
        return dropped;
    }

    /*** Move the oldest scan out of the queue; overflow can no longer reach it. ***/
    bool claim()
    {
        if (pending_valid_ || queue_.empty()) return false;
        pending_ = std::move(queue_.front());
        queue_.pop_front();
        pending_valid_ = true;
        return true;
    }

    bool has_pending() const { return pending_valid_; }
    const LidarScan<CloudPtr> &pending() const { return pending_; }

    void release_pending()
    {
        pending_ = LidarScan<CloudPtr>{};
        pending_valid_ = false;
    }

    void clear()
    {
        queue_.clear();
        release_pending();
    }

    bool empty() const { return queue_.empty(); }
    std::size_t size() const { return queue_.size(); }
    std::size_t capacity() const { return capacity_; }

private:
    std::size_t capacity_;
    std::deque<LidarScan<CloudPtr>> queue_;
    LidarScan<CloudPtr> pending_{};
    bool pending_valid_{false};
};

}  // namespace fast_lio
