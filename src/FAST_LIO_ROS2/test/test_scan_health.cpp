#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include <fast_lio/scan_health.hpp>
#include <fast_lio/recovery_gate.hpp>

using fast_lio::LidarScan;
using fast_lio::LidarScanQueue;
using fast_lio::ScanHealthMonitor;
using fast_lio::ScanOutcome;

namespace
{

/*** Stand-in for PointCloudXYZI::Ptr: the queue only moves the handle around, so a
     shared_ptr<int> is enough to tell one scan from another. ***/
using CloudPtr = std::shared_ptr<int>;
using Queue = LidarScanQueue<CloudPtr>;

LidarScan<CloudPtr> makeScan(int id, double timestamp)
{
    LidarScan<CloudPtr> scan;
    scan.cloud = std::make_shared<int>(id);
    scan.timestamp = timestamp;
    scan.received_at = std::chrono::steady_clock::now();
    return scan;
}

}  // namespace

/*** One bad frame that trips several gates must still count as a single fault,
     otherwise a single dropout latches a hard fault on its own. ***/
TEST(ScanHealthMonitor, SeveralReasonsInOneFrameCountOnce)
{
    ScanHealthMonitor monitor;
    const auto decision = monitor.evaluate(
        fast_lio::SCAN_FAULT_LIDAR_GAP | fast_lio::SCAN_FAULT_IMU_GAP |
        fast_lio::SCAN_FAULT_MEASUREMENT_GATE | fast_lio::SCAN_FAULT_EMPTY_SCAN, 1.0);

    EXPECT_EQ(decision.outcome, ScanOutcome::REJECTED);
    EXPECT_EQ(decision.consecutive_faults, 1);
    EXPECT_FALSE(decision.hard_fault);
    EXPECT_EQ(monitor.consecutive_faults(), 1);
    EXPECT_FALSE(monitor.hard_fault());
}

TEST(ScanHealthMonitor, RequiresTenBadFramesAndOneSecondOfSourceTime)
{
    ScanHealthMonitor monitor;
    for (int frame = 1; frame <= 10; ++frame)
    {
        const auto decision = monitor.evaluate(
            fast_lio::SCAN_FAULT_MEASUREMENT_GATE, 1.0 + (frame - 1) * 0.1);
        EXPECT_EQ(decision.outcome, ScanOutcome::REJECTED) << "frame " << frame;
        EXPECT_EQ(decision.consecutive_faults, frame);
        EXPECT_FALSE(decision.hard_fault) << "latched before one second on frame " << frame;
    }

    const auto confirmed = monitor.evaluate(fast_lio::SCAN_FAULT_MEASUREMENT_GATE, 2.0);
    EXPECT_EQ(confirmed.outcome, ScanOutcome::RELOCALIZATION_REQUIRED);
    EXPECT_EQ(confirmed.consecutive_faults, 11);
    EXPECT_TRUE(confirmed.hard_fault);
    EXPECT_TRUE(monitor.hard_fault());
}

TEST(ScanHealthMonitor, GoodFrameClearsTheConsecutiveCounter)
{
    ScanHealthMonitor monitor;
    monitor.evaluate(fast_lio::SCAN_FAULT_LIDAR_GAP, 1.0);
    monitor.evaluate(fast_lio::SCAN_FAULT_LIDAR_GAP, 1.1);
    ASSERT_EQ(monitor.consecutive_faults(), 2);

    const auto good = monitor.evaluate(fast_lio::SCAN_FAULT_NONE, 1.2);
    EXPECT_EQ(good.outcome, ScanOutcome::ACCEPTED);
    EXPECT_TRUE(good.healthy());
    EXPECT_TRUE(good.may_update_map());
    EXPECT_EQ(monitor.consecutive_faults(), 0);

    /*** A healthy source frame breaks the run; following bad frames start over. ***/
    for (int frame = 1; frame <= 10; ++frame)
    {
        EXPECT_FALSE(monitor.evaluate(
            fast_lio::SCAN_FAULT_LIDAR_GAP, 1.2 + frame * 0.1).hard_fault)
            << "frame " << frame;
    }
    EXPECT_TRUE(monitor.evaluate(fast_lio::SCAN_FAULT_LIDAR_GAP, 2.3).hard_fault);
}

TEST(ScanHealthMonitor, DuplicateAndOldSourceScansDoNotAdvanceFaultRun)
{
    ScanHealthMonitor monitor;
    const auto first = monitor.evaluate(fast_lio::SCAN_FAULT_EMPTY_SCAN, 1.0);
    ASSERT_EQ(first.consecutive_faults, 1);

    const auto duplicate = monitor.evaluate(fast_lio::SCAN_FAULT_MEASUREMENT_GATE, 1.0);
    EXPECT_EQ(duplicate.outcome, ScanOutcome::DUPLICATE);
    EXPECT_EQ(duplicate.consecutive_faults, 1);

    const auto old = monitor.evaluate(fast_lio::SCAN_FAULT_EMPTY_SCAN, 0.9);
    EXPECT_EQ(old.outcome, ScanOutcome::DUPLICATE);
    EXPECT_EQ(old.consecutive_faults, 1);

    const auto next = monitor.evaluate(fast_lio::SCAN_FAULT_MEASUREMENT_GATE, 1.1);
    EXPECT_EQ(next.outcome, ScanOutcome::REJECTED);
    EXPECT_EQ(next.consecutive_faults, 2);
}

/*** A rejected frame is the degraded path: report unhealthy and keep the scan out of
     the map, while the caller still publishes the propagated odometry. ***/
TEST(ScanHealthMonitor, QueueDropAloneKeepsTheCurrentCompleteScanAccepted)
{
    ScanHealthMonitor monitor;
    const auto decision = monitor.evaluate(fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP, 1.0);

    EXPECT_EQ(decision.reasons, fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP);
    EXPECT_EQ(decision.outcome, ScanOutcome::ACCEPTED);
    EXPECT_TRUE(decision.healthy());
    EXPECT_TRUE(decision.may_update_map());
    EXPECT_FALSE(decision.hard_fault);
    EXPECT_EQ(decision.consecutive_faults, 0);
}

TEST(ScanHealthMonitor, MeasuredLidarGapAboveHalfSecondStillRejects)
{
    constexpr double threshold = 0.50;
    EXPECT_FALSE(fast_lio::lidar_gap_exceeds_threshold(threshold, threshold));
    EXPECT_TRUE(fast_lio::lidar_gap_exceeds_threshold(
        std::nextafter(threshold, 1.0), threshold));

    ScanHealthMonitor monitor;
    const auto decision = monitor.evaluate(
        fast_lio::SCAN_FAULT_LIDAR_GAP | fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP, 1.0);
    EXPECT_EQ(decision.outcome, ScanOutcome::REJECTED);
    EXPECT_FALSE(decision.may_update_map());
}

TEST(ScanHealthMonitor, RejectedFrameIsUnhealthyAndMayNotUpdateTheMap)
{
    ScanHealthMonitor monitor;
    const auto rejected = monitor.evaluate(fast_lio::SCAN_FAULT_EMPTY_SCAN, 1.0);
    EXPECT_FALSE(rejected.healthy());
    EXPECT_FALSE(rejected.may_update_map());

    const auto accepted = monitor.evaluate(fast_lio::SCAN_FAULT_NONE, 1.1);
    EXPECT_TRUE(accepted.healthy());
    EXPECT_TRUE(accepted.may_update_map());
}

TEST(ScanHealthMonitor, TimeRegressionAndNonFiniteStateAreFatal)
{
    ScanHealthMonitor regression;
    const auto fatal = regression.evaluate(fast_lio::SCAN_FAULT_TIME_REGRESSION, 1.0);
    EXPECT_EQ(fatal.outcome, ScanOutcome::FATAL);
    EXPECT_TRUE(fatal.hard_fault);
    EXPECT_FALSE(fatal.may_update_map());
    /*** Fatal input is isolated immediately, never added to the ordinary run. ***/
    EXPECT_EQ(fatal.consecutive_faults, 0);

    ScanHealthMonitor non_finite;
    EXPECT_EQ(
        non_finite.evaluate(
            fast_lio::SCAN_FAULT_MEASUREMENT_GATE | fast_lio::SCAN_FAULT_NON_FINITE_STATE,
            1.0)
            .outcome,
        ScanOutcome::FATAL);
}

TEST(ScanHealthMonitor, ResetClearsTheLatch)
{
    ScanHealthMonitor monitor;
    for (int frame = 0; frame <= 10; ++frame) {
        monitor.evaluate(fast_lio::SCAN_FAULT_LIDAR_GAP, 1.0 + frame * 0.1);
    }
    ASSERT_TRUE(monitor.hard_fault());

    monitor.reset();
    EXPECT_FALSE(monitor.hard_fault());
    EXPECT_EQ(monitor.consecutive_faults(), 0);
}

TEST(ScanFaultReasonText, ListsEveryRaisedReason)
{
    EXPECT_EQ(fast_lio::scan_fault_reason_text(fast_lio::SCAN_FAULT_NONE), "none");
    EXPECT_EQ(
        fast_lio::scan_fault_reason_text(
            fast_lio::SCAN_FAULT_LIDAR_GAP | fast_lio::SCAN_FAULT_IMU_GAP),
        "lidar gap, imu gap");
    EXPECT_EQ(
        fast_lio::scan_fault_reason_text(fast_lio::SCAN_FAULT_LIDAR_QUEUE_DROP),
        "lidar queue drop");
}

/*** The core of P1-1: the scan handed to the filter lives in the pending slot, so an
     overflowing callback can only drop scans nobody has claimed. ***/
TEST(LidarScanQueue, PendingScanSurvivesOverflow)
{
    Queue queue(3);
    queue.push(makeScan(1, 100.0));
    ASSERT_TRUE(queue.claim());
    ASSERT_TRUE(queue.has_pending());

    const auto claimed_at = queue.pending().received_at;
    EXPECT_EQ(*queue.pending().cloud, 1);

    /*** Overflow the queue several times over while scan 1 waits for IMU. ***/
    std::size_t dropped = 0;
    for (int id = 2; id <= 9; ++id)
    {
        dropped += queue.push(makeScan(id, 100.0 + id));
    }
    EXPECT_GT(dropped, 0u);

    EXPECT_TRUE(queue.has_pending());
    EXPECT_EQ(*queue.pending().cloud, 1);
    EXPECT_DOUBLE_EQ(queue.pending().timestamp, 100.0);
    /*** The pending scan keeps its own arrival time; it must not inherit a newer
         scan's, or the backlog measurement reports the wrong frame. ***/
    EXPECT_EQ(queue.pending().received_at, claimed_at);
    EXPECT_LE(queue.size(), queue.capacity());
}

TEST(LidarScanQueue, PushReportsDroppedScansOnlyWhenOverCapacity)
{
    Queue queue(2);
    EXPECT_EQ(queue.push(makeScan(1, 1.0)), 0u);
    EXPECT_EQ(queue.push(makeScan(2, 2.0)), 0u);
    EXPECT_EQ(queue.push(makeScan(3, 3.0)), 1u);
    EXPECT_EQ(queue.size(), 2u);

    /*** The oldest unclaimed scan is the one that goes. ***/
    ASSERT_TRUE(queue.claim());
    EXPECT_EQ(*queue.pending().cloud, 2);
}

TEST(LidarScanQueue, ClaimIsSingleSlotAndReleaseFreesIt)
{
    Queue queue(3);
    EXPECT_FALSE(queue.claim()) << "nothing to claim from an empty queue";

    queue.push(makeScan(1, 1.0));
    queue.push(makeScan(2, 2.0));
    ASSERT_TRUE(queue.claim());
    EXPECT_FALSE(queue.claim()) << "a second claim must not overwrite the pending scan";
    EXPECT_EQ(queue.size(), 1u);

    queue.release_pending();
    EXPECT_FALSE(queue.has_pending());
    ASSERT_TRUE(queue.claim());
    EXPECT_EQ(*queue.pending().cloud, 2);
}

/*** Timestamp regression clears everything, including the claimed scan: it belongs to
     the old time axis. ***/
TEST(LidarScanQueue, ClearDropsPendingAndQueuedScans)
{
    Queue queue(3);
    queue.push(makeScan(1, 1.0));
    queue.push(makeScan(2, 2.0));
    ASSERT_TRUE(queue.claim());

    queue.clear();
    EXPECT_FALSE(queue.has_pending());
    EXPECT_TRUE(queue.empty());
    EXPECT_FALSE(queue.claim());
}

TEST(LidarScanQueue, CapacityIsAtLeastOne)
{
    Queue queue(0);
    EXPECT_EQ(queue.capacity(), 1u);
    EXPECT_EQ(queue.push(makeScan(1, 1.0)), 0u);
    EXPECT_EQ(queue.push(makeScan(2, 2.0)), 1u);
    ASSERT_TRUE(queue.claim());
    EXPECT_EQ(*queue.pending().cloud, 2);
}


TEST(RecoveryGate, RequiresFiveGoodFramesAndHalfSecond)
{
    fast_lio::RecoveryGate gate;
    gate.start();
    for (int frame = 0; frame < 5; ++frame) {
        const auto result = gate.observe_candidate(
            true, 0.05, 0.02, 10.0 + 0.1 * frame);
        EXPECT_EQ(result, fast_lio::RecoveryGate::Result::WAITING);
    }
    EXPECT_EQ(gate.observe_candidate(true, 0.05, 0.02, 10.5),
        fast_lio::RecoveryGate::Result::READY);
    EXPECT_FALSE(gate.active());
}

TEST(RecoveryGate, InvalidFrameRestartsConsecutiveStabilityWindow)
{
    using Clock = fast_lio::RecoveryGate::Clock;
    fast_lio::RecoveryGate gate;
    const auto start = Clock::time_point{};
    gate.start();
    gate.observe(true, 0.01, 0.01, start);
    gate.observe(true, 0.01, 0.01, start + std::chrono::milliseconds(100));
    EXPECT_EQ(gate.observe(false, 0.0, 0.0, start + std::chrono::milliseconds(200)),
        fast_lio::RecoveryGate::Result::WAITING);
    EXPECT_EQ(gate.consecutive_frames(), 0u);
    EXPECT_EQ(gate.observe(true, 0.01, 0.01, start + std::chrono::milliseconds(300)),
        fast_lio::RecoveryGate::Result::WAITING);
}

TEST(RecoveryGate, PredictionOutlierRestartsStabilityWindowWithoutRelocalizing)
{
    fast_lio::RecoveryGate gate;
    gate.start();
    EXPECT_EQ(gate.observe_candidate(true, 0.11, 0.0, 1.0),
        fast_lio::RecoveryGate::Result::WAITING);
    EXPECT_EQ(gate.consecutive_frames(), 0u);
    EXPECT_EQ(gate.observe_candidate(true, 0.0, 0.0, 1.1),
        fast_lio::RecoveryGate::Result::WAITING);
    EXPECT_EQ(gate.consecutive_frames(), 1u);
}
