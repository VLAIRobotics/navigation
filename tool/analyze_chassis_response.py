#!/usr/bin/env python3
"""Estimate chassis CAN speed scales from a rosbag2 recording.

The tool is read-only: it never publishes a command or talks to CAN.  It
expects the isolated pulse protocol documented in ``src-architecture-index``
and produces a recommendation only after every configured command level has
three independent bidirectional pulses.
"""

import argparse
import bisect
import json
import statistics
import sys


EXPECTED_LINEAR_LEVELS = (0.15, 0.25, 0.35)
EXPECTED_ANGULAR_LEVELS = (0.10, 0.20, 0.30)
LEVEL_TOLERANCE = 0.02
LINEAR_MOVING_THRESHOLD = 0.03
ANGULAR_MOVING_THRESHOLD = 0.05
CROSS_AXIS_LINEAR_THRESHOLD = 0.03
CROSS_AXIS_ANGULAR_THRESHOLD = 0.05
MIN_PULSE_SECONDS = 3.0
MIN_ZERO_SECONDS = 3.0
DURATION_TOLERANCE = 1.0e-6


def _median(values):
    return statistics.median(values) if values else None


def _command_changed(previous, current):
    return (
        abs(current[0] - previous[0]) > 0.01 or
        abs(current[1] - previous[1]) > 0.01
    )


def _isolated_axis(linear_command, angular_command):
    if abs(angular_command) <= 0.02 and abs(linear_command) > 0.05:
        return "linear"
    if abs(linear_command) <= 0.02 and abs(angular_command) > 0.02:
        return "angular"
    return None


def _is_zero_command(linear_command, angular_command):
    return abs(linear_command) <= 0.02 and abs(angular_command) <= 0.02


def _build_command_segments(commands):
    """Collapse repeated commands into timestamped command-state segments."""
    segments = []
    if not commands:
        return segments

    start = commands[0][0]
    linear = commands[0][1]
    angular = commands[0][2]
    for timestamp, next_linear, next_angular in commands[1:]:
        current = (next_linear, next_angular)
        if _command_changed((linear, angular), current):
            segments.append({
                "start": start,
                "end": timestamp,
                "linear": linear,
                "angular": angular,
            })
            start = timestamp
            linear, angular = current
    segments.append({
        "start": start,
        "end": float("inf"),
        "linear": linear,
        "angular": angular,
    })
    return segments


def _level_for_command(command, expected_levels):
    magnitude = abs(command)
    if not expected_levels:
        return None
    level = min(expected_levels, key=lambda expected: abs(magnitude - expected))
    tolerance = max(LEVEL_TOLERANCE, level * 0.10)
    return level if abs(magnitude - level) <= tolerance else None


def _direction_report(samples, expected_levels):
    grouped = {"positive": [], "negative": []}
    for command, velocity in samples:
        if abs(command) < 1e-9:
            continue
        grouped["positive" if command > 0.0 else "negative"].append(
            (abs(command), velocity))

    report = {}
    for direction, values in grouped.items():
        ratios = [abs(velocity) / command for command, velocity in values if command > 0.0]
        level_counts = {f"{level:.2f}": 0 for level in expected_levels}
        for command, _ in values:
            level = _level_for_command(command, expected_levels)
            if level is not None:
                level_counts[f"{level:.2f}"] += 1
        report[direction] = {
            "samples": len(values),
            "median_command": _median([item[0] for item in values]),
            "median_velocity": _median([abs(item[1]) for item in values]),
            "median_gain": _median(ratios),
            "peak_velocity": max((abs(item[1]) for item in values), default=None),
            "min_moving_command": None,
            "level_counts": level_counts,
        }
    return report


def _pulse_report(pulse, axis):
    """Return a stable per-pulse sample, or None if no settled odometry exists."""
    if not pulse["steady"]:
        return None
    command = pulse["linear_command"] if axis == "linear" else pulse["angular_command"]
    velocity_index = 0 if axis == "linear" else 1
    response = _median([sample[velocity_index] for sample in pulse["steady"]])
    cross_index = 1 if axis == "linear" else 0
    cross_response = _median([sample[cross_index] for sample in pulse["steady"]])
    return {
        "command": command,
        "response": response,
        "cross_response": cross_response,
        "axis": axis,
        "start": pulse["start"],
    }


def _direction_mismatch(sample, moving_threshold):
    return (
        abs(sample["response"]) >= moving_threshold and
        sample["command"] * sample["response"] < 0.0
    )


def analyze_samples(
    commands,
    odometry,
    old_linear_scale,
    old_angular_scale,
    settle_seconds=1.0,
    max_command_age=0.15,
    min_samples=3,
    max_observed_linear=0.5,
    max_observed_angular=0.5,
):
    """Analyze timestamped command/odom tuples without importing ROS.

    ``commands`` contains ``(time_s, linear_x, angular_z)`` and ``odometry``
    contains ``(time_s, linear_x, angular_z)``.  A pulse is counted once using
    the median settled response; repeated odometry messages cannot satisfy the
    independent-pulse requirement.  The returned dictionary is JSON
    serializable and includes a ``safe_recommendation`` boolean.
    """
    if not commands or not odometry:
        raise ValueError("both command and odometry samples are required")
    if settle_seconds < 0.0 or max_command_age < 0.0 or min_samples < 1:
        raise ValueError("settle time, command age, and sample count must be valid")

    commands = sorted(commands)
    odometry = sorted(odometry)
    segments = _build_command_segments(commands)
    recording_end = max(commands[-1][0], odometry[-1][0])
    segment_starts = [segment["start"] for segment in segments]
    command_times = [item[0] for item in commands]

    pulses = []
    protocol_rejection_reasons = set()
    for index, segment in enumerate(segments):
        axis = _isolated_axis(segment["linear"], segment["angular"])
        if axis is None:
            continue
        segment_end = segment["end"]
        effective_segment_end = (
            recording_end if segment_end == float("inf") else segment_end)
        pulse_duration = effective_segment_end - segment["start"]
        protocol_valid = pulse_duration + DURATION_TOLERANCE >= MIN_PULSE_SECONDS
        if not protocol_valid:
            protocol_rejection_reasons.add("short_pulse")
        if segment_end == float("inf"):
            protocol_rejection_reasons.add("incomplete_final_pulse")

        zero_segment = segments[index + 1] if index + 1 < len(segments) else None
        has_zero_interval = (
            zero_segment is not None and
            _is_zero_command(zero_segment["linear"], zero_segment["angular"])
        )
        if not has_zero_interval:
            protocol_valid = False
            protocol_rejection_reasons.add("missing_zero_interval")
            zero_start = effective_segment_end
            zero_end = effective_segment_end
        else:
            zero_start = zero_segment["start"]
            zero_segment_end = zero_segment["end"]
            zero_end = (
                recording_end if zero_segment_end == float("inf")
                else zero_segment_end
            )
            zero_duration = zero_end - zero_start
            if zero_duration + DURATION_TOLERANCE < MIN_ZERO_SECONDS:
                protocol_valid = False
                protocol_rejection_reasons.add("short_zero_interval")

        pulses.append({
            "segment_index": index,
            "start": segment["start"],
            "pulse_end": effective_segment_end,
            "zero_start": zero_start,
            "safety_end": zero_end,
            "linear_command": segment["linear"],
            "angular_command": segment["angular"],
            "protocol_valid": protocol_valid,
            "steady": [],
            "safety": [],
            "coast": [],
        })

    pulse_starts = [pulse["start"] for pulse in pulses]
    for timestamp, linear_velocity, angular_velocity in odometry:
        segment_index = bisect.bisect_right(segment_starts, timestamp) - 1
        command_index = bisect.bisect_right(command_times, timestamp) - 1
        if segment_index < 0 or command_index < 0:
            continue
        if timestamp - commands[command_index][0] > max_command_age:
            continue
        pulse_index = bisect.bisect_right(pulse_starts, timestamp) - 1
        if pulse_index < 0:
            continue
        pulse = pulses[pulse_index]
        if timestamp >= pulse["safety_end"]:
            continue
        sample = (linear_velocity, angular_velocity)
        pulse["safety"].append(sample)
        if (segment_index == pulse["segment_index"] and
                timestamp - pulse["start"] >= settle_seconds):
            pulse["steady"].append(sample)
        elif timestamp >= pulse["zero_start"]:
            pulse["coast"].append(sample)

    linear_samples = []
    angular_samples = []
    direction_mismatches = []
    cross_axis_violations = []
    no_motion = []
    observed_linear = []
    observed_angular = []
    usable_pulses = 0
    for pulse in pulses:
        observed_linear.extend(abs(sample[0]) for sample in pulse["safety"])
        observed_angular.extend(abs(sample[1]) for sample in pulse["safety"])
        if not pulse["protocol_valid"]:
            continue
        axis = _isolated_axis(pulse["linear_command"], pulse["angular_command"])
        report = _pulse_report(pulse, axis)
        if report is None:
            protocol_rejection_reasons.add("insufficient_steady_samples")
            continue
        if not pulse["coast"]:
            protocol_rejection_reasons.add("insufficient_coast_samples")
            continue
        usable_pulses += 1
        moving_threshold = LINEAR_MOVING_THRESHOLD if axis == "linear" else ANGULAR_MOVING_THRESHOLD
        cross_threshold = CROSS_AXIS_ANGULAR_THRESHOLD if axis == "linear" else CROSS_AXIS_LINEAR_THRESHOLD
        if abs(report["response"]) < moving_threshold:
            no_motion.append({"axis": axis, "command": report["command"], "start": report["start"]})
        if _direction_mismatch(report, moving_threshold):
            direction_mismatches.append({
                "axis": axis, "command": report["command"],
                "response": report["response"], "start": report["start"],
            })
        if any(abs(sample[1 if axis == "linear" else 0]) > cross_threshold
               for sample in pulse["safety"]):
            cross_axis_violations.append({
                "axis": axis, "command": report["command"], "start": report["start"],
            })
        target = linear_samples if axis == "linear" else angular_samples
        target.append((report["command"], report["response"]))

    linear = _direction_report(linear_samples, EXPECTED_LINEAR_LEVELS)
    angular = _direction_report(angular_samples, EXPECTED_ANGULAR_LEVELS)

    def moving_threshold(report, threshold, samples):
        by_direction = {"positive": [], "negative": []}
        for command, velocity in samples:
            if abs(velocity) >= threshold:
                by_direction["positive" if command > 0.0 else "negative"].append(abs(command))
        for direction in by_direction:
            report[direction]["min_moving_command"] = min(by_direction[direction], default=None)

    moving_threshold(linear, LINEAR_MOVING_THRESHOLD, linear_samples)
    moving_threshold(angular, ANGULAR_MOVING_THRESHOLD, angular_samples)

    def gain_pair(report):
        values = [report[direction]["median_gain"] for direction in ("positive", "negative")]
        return values if all(value is not None for value in values) else None

    linear_gains = gain_pair(linear)
    angular_gains = gain_pair(angular)
    linear_asymmetry = (
        max(linear_gains) / min(linear_gains)
        if linear_gains and min(linear_gains) > 0.0 else None
    )
    angular_asymmetry = (
        max(angular_gains) / min(angular_gains)
        if angular_gains and min(angular_gains) > 0.0 else None
    )

    def missing_levels(report, expected_levels):
        missing = []
        for direction in ("positive", "negative"):
            for level in expected_levels:
                key = f"{level:.2f}"
                if report[direction]["level_counts"][key] < min_samples:
                    missing.append(f"{direction} {key}")
        return missing

    missing_linear_levels = missing_levels(linear, EXPECTED_LINEAR_LEVELS)
    missing_angular_levels = missing_levels(angular, EXPECTED_ANGULAR_LEVELS)
    enough_samples = not missing_linear_levels and not missing_angular_levels
    safety_sample_count = len(observed_linear) + len(observed_angular)
    safe_peaks = safety_sample_count > 0 and (
        max(observed_linear, default=0.0) <= max_observed_linear and
        max(observed_angular, default=0.0) <= max_observed_angular
    )
    symmetric = (
        linear_asymmetry is not None and linear_asymmetry <= 1.20 and
        angular_asymmetry is not None and angular_asymmetry <= 1.20
    )
    recommendation = None
    if (
        enough_samples and safe_peaks and symmetric and not no_motion and
        not direction_mismatches and not cross_axis_violations and
        not protocol_rejection_reasons
    ):
        recommendation = {
            # The larger directional gain is conservative: the same CAN raw
            # value will not exceed the requested physical velocity.
            "max_linear_speed": old_linear_scale * max(linear_gains),
            "max_angular_speed": old_angular_scale * max(angular_gains),
        }

    rejection_reasons = []
    rejection_reasons.extend(sorted(protocol_rejection_reasons))
    if not enough_samples:
        rejection_reasons.append("insufficient_direction_samples")
        rejection_reasons.append("insufficient_pulse_repetitions")
        if missing_linear_levels:
            rejection_reasons.append("missing_linear_pulse_levels")
        if missing_angular_levels:
            rejection_reasons.append("missing_angular_pulse_levels")
    if safety_sample_count == 0:
        rejection_reasons.append("insufficient_safety_samples")
    elif not safe_peaks:
        rejection_reasons.append("observed_velocity_exceeds_safety_limit")
    if no_motion:
        rejection_reasons.append("no_motion_detected")
    if direction_mismatches:
        rejection_reasons.append("response_direction_mismatch")
    if cross_axis_violations:
        rejection_reasons.append("cross_axis_motion_exceeds_limit")
    if linear_gains and linear_asymmetry is not None and linear_asymmetry > 1.20:
        rejection_reasons.append("linear_directional_asymmetry_exceeds_20_percent")
    if angular_gains and angular_asymmetry is not None and angular_asymmetry > 1.20:
        rejection_reasons.append("angular_directional_asymmetry_exceeds_20_percent")
    if enough_samples and (linear_gains is None or angular_gains is None):
        rejection_reasons.append("insufficient_response_samples")

    return {
        "safe_recommendation": recommendation is not None,
        "rejection_reasons": rejection_reasons,
        "linear": linear,
        "angular": angular,
        "linear_asymmetry_ratio": linear_asymmetry,
        "angular_asymmetry_ratio": angular_asymmetry,
        "peak_observed_linear_velocity": max(observed_linear, default=0.0),
        "peak_observed_angular_velocity": max(observed_angular, default=0.0),
        "safety_sample_count": safety_sample_count,
        "pulse_count": len(pulses),
        "protocol_valid_pulse_count": sum(
            1 for pulse in pulses if pulse["protocol_valid"]),
        "usable_pulse_count": usable_pulses,
        "protocol_rejection_reasons": sorted(protocol_rejection_reasons),
        "missing_linear_pulse_levels": missing_linear_levels,
        "missing_angular_pulse_levels": missing_angular_levels,
        "direction_mismatch_count": len(direction_mismatches),
        "cross_axis_violation_count": len(cross_axis_violations),
        "no_motion_count": len(no_motion),
        "recommendation": recommendation,
    }


def read_bag(path, command_topic, odometry_topic):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(f"ROS 2 Python modules are required: {error}") from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in (command_topic, odometry_topic) if topic not in topics]
    if missing:
        raise ValueError(f"bag has no required topic(s): {', '.join(missing)}")
    command_type = get_message(topics[command_topic])
    odometry_type = get_message(topics[odometry_topic])
    commands = []
    odometry = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns / 1e9
        if topic == command_topic:
            message = deserialize_message(serialized, command_type)
            commands.append((timestamp, message.linear.x, message.angular.z))
        elif topic == odometry_topic:
            message = deserialize_message(serialized, odometry_type)
            odometry.append((
                timestamp,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            ))
    return commands, odometry


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--command-topic", default="/cmd_vel")
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--old-linear-scale", type=float, default=0.5)
    parser.add_argument("--old-angular-scale", type=float, default=1.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--min-samples", type=int, default=3)
    args = parser.parse_args(argv)
    if (args.old_linear_scale <= 0.0 or args.old_angular_scale <= 0.0 or
            args.settle_seconds < 0.0 or args.min_samples < 1):
        parser.error("scales must be positive, settle time non-negative, and min-samples >= 1")
    try:
        commands, odometry = read_bag(args.bag, args.command_topic, args.odometry_topic)
        report = analyze_samples(
            commands, odometry, args.old_linear_scale, args.old_angular_scale,
            settle_seconds=args.settle_seconds, min_samples=args.min_samples,
        )
    except (RuntimeError, ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["safe_recommendation"] else 1


if __name__ == "__main__":
    sys.exit(main())
