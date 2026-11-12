#!/usr/bin/env python3
"""Summarize FastDEM terrain features from a rosbag2 recording.

The input bag must contain /fastdem/postprocess/gridmap.  The report is
JSON on stdout so it can be archived with the private task review.
"""

import argparse
import json
import math
import sys
from collections import defaultdict


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def layer_data(message, name):
    try:
        index = message.layers.index(name)
    except ValueError:
        return None
    return message.data[index].data


def grid_shape(message, data):
    if message.data:
        dimensions = message.data[0].layout.dim
        if len(dimensions) == 2:
            return int(dimensions[0].size), int(dimensions[1].size)
    rows = max(1, cpp_llround(message.info.length_x / message.info.resolution))
    cols = max(1, cpp_llround(message.info.length_y / message.info.resolution))
    return rows, cols


def cpp_llround(value):
    """Match C++ llround: halfway cases are rounded away from zero."""
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def world_key(message, physical_index, rows, cols):
    # nanogrid/Eigen stores matrices column-major: physical index is
    # column * rows + row.  nanogrid's map-frame index runs in the opposite
    # direction from the buffer index on both axes.
    row = physical_index % rows
    col = physical_index // rows
    logical_row = (row - message.outer_start_index) % rows
    logical_col = (col - message.inner_start_index) % cols
    x = (message.info.pose.position.x + message.info.length_x / 2.0 -
         message.info.resolution / 2.0 - logical_row * message.info.resolution)
    y = (message.info.pose.position.y + message.info.length_y / 2.0 -
         message.info.resolution / 2.0 - logical_col * message.info.resolution)
    scale = max(message.info.resolution, 1e-9)
    return cpp_llround(x / scale), cpp_llround(y / scale)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--topic", default="/fastdem/postprocess/gridmap")
    parser.add_argument("--phase-topic", default="/fastdem/terrain_baseline_phase")
    parser.add_argument("--min-observations", type=float, default=3.0)
    parser.add_argument("--max-observation-age-frames", type=float, default=3.0)
    parser.add_argument("--slope-lethal", type=float, default=30.0)
    parser.add_argument("--residual-lethal", type=float, default=0.15)
    parser.add_argument("--roughness-lethal", type=float, default=0.15)
    parser.add_argument("--min-confident-cells", type=int, default=20)
    parser.add_argument("--max-lethal-fraction", type=float, default=0.90)
    args = parser.parse_args()
    if (args.min_confident_cells < 1 or args.max_lethal_fraction <= 0.0 or
            args.max_lethal_fraction > 1.0):
        parser.error("--min-confident-cells must be >= 1 and --max-lethal-fraction must be in (0, 1]")

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        print(f"ROS 2 Python modules are required: {error}", file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if args.topic not in topics:
        print(f"bag has no {args.topic}; available topics: {sorted(topics)}", file=sys.stderr)
        return 2
    message_type = get_message(topics[args.topic])
    phase_type = get_message(topics[args.phase_topic]) if args.phase_topic in topics else None

    features = ("slope", "step_residual", "roughness")
    values = defaultdict(list)
    lethal_cells = defaultdict(int)
    total_frames = 0
    first_ns = None
    last_ns = None
    runs = {}
    feature_runs = {name: {} for name in features}
    max_run = {name: {"frames": 0, "duration_s": 0.0} for name in features}
    combined_max_run = {"frames": 0, "duration_s": 0.0}
    phase = "unknown"
    phase_reports = defaultdict(lambda: {
        "frames": 0, "values": defaultdict(list), "lethal_cell_frame_count": 0,
        "confident_cells": [], "runs": {}, "generation": 0,
        "accepted_frames": 0, "rejected_frames": 0,
        "max_run": {"frames": 0, "duration_s": 0.0},
    })
    missing_confidence_frames = 0
    repeated_or_rollback_frames = 0
    low_confidence_cells = 0
    confident_cells_per_frame = []
    unique_frame = 0
    accepted_frame = 0
    rejected_frames = 0
    rejection_reasons = defaultdict(int)
    last_unique_timestamp = None
    phase_generation = 0

    def reset_continuity(phase_report):
        runs.clear()
        for feature_run in feature_runs.values():
            feature_run.clear()
        phase_report["runs"].clear()

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == args.phase_topic and phase_type is not None:
            phase = deserialize_message(serialized, phase_type).data.strip() or "unknown"
            phase_generation += 1
            continue
        if topic != args.topic:
            continue
        message = deserialize_message(serialized, message_type)
        total_frames += 1
        timestamp = stamp_ns(message)
        if last_unique_timestamp is not None and timestamp <= last_unique_timestamp:
            repeated_or_rollback_frames += 1
            if timestamp < last_unique_timestamp:
                runs.clear()
                for feature_run in feature_runs.values():
                    feature_run.clear()
                for report in phase_reports.values():
                    report["runs"].clear()
            continue
        last_unique_timestamp = timestamp
        unique_frame += 1
        phase_report = phase_reports[phase]
        phase_report["frames"] += 1
        if phase_report["generation"] != phase_generation:
            phase_report["runs"] = {}
            phase_report["generation"] = phase_generation
        first_ns = timestamp if first_ns is None else min(first_ns, timestamp)
        last_ns = timestamp if last_ns is None else max(last_ns, timestamp)
        arrays = {name: layer_data(message, name) for name in features}
        points = layer_data(message, "n_points")
        ages = layer_data(message, "observation_age")
        if any(array is None for array in arrays.values()) or points is None or ages is None:
            missing_confidence_frames += 1
            rejected_frames += 1
            phase_report["rejected_frames"] += 1
            rejection_reasons["missing_confidence_layers"] += 1
            reset_continuity(phase_report)
            continue
        rows, cols = grid_shape(message, arrays["slope"])
        size = min(len(arrays[name]) for name in features)
        size = min(size, len(points), len(ages), rows * cols)
        confident = 0
        frame_lethal_cells = set()
        feature_lethal_cells = {name: set() for name in features}
        frame_feature_values = {name: [] for name in features}
        for index in range(size):
            has_feature = any(math.isfinite(arrays[name][index]) for name in features)
            if not has_feature:
                continue
            confident_cell = (
                math.isfinite(points[index])
                and points[index] >= args.min_observations
                and math.isfinite(ages[index])
                and 0.0 <= ages[index] <= args.max_observation_age_frames
            )
            if not confident_cell:
                low_confidence_cells += 1
                continue
            confident += 1
            key = world_key(message, index, rows, cols)
            cell_lethal = False
            for name, lethal_threshold in (
                ("slope", args.slope_lethal),
                ("step_residual", args.residual_lethal),
                ("roughness", args.roughness_lethal),
            ):
                value = arrays[name][index]
                if not math.isfinite(value):
                    continue
                frame_feature_values[name].append(float(value))
                if value < lethal_threshold:
                    continue
                feature_lethal_cells[name].add(key)
                cell_lethal = True
            if cell_lethal:
                frame_lethal_cells.add(key)
        lethal_fraction = (1.0 if confident == 0 else
                           len(frame_lethal_cells) / float(confident))
        rejected_for_confidence = confident < args.min_confident_cells
        rejected_for_lethal_fraction = lethal_fraction > args.max_lethal_fraction

        # Keep raw baseline statistics for every parseable frame.  Acceptance
        # only controls the continuity state below; it must not hide frames
        # that explain why the runtime rejected them.
        for name in features:
            values[name].extend(frame_feature_values[name])
            phase_report["values"][name].extend(frame_feature_values[name])
            lethal_cells[name] += len(feature_lethal_cells[name])
        phase_report["lethal_cell_frame_count"] += len(frame_lethal_cells)
        confident_cells_per_frame.append(confident)
        phase_report["confident_cells"].append(confident)

        if rejected_for_confidence or rejected_for_lethal_fraction:
            rejected_frames += 1
            phase_report["rejected_frames"] += 1
            if rejected_for_confidence and rejected_for_lethal_fraction:
                rejection_reasons["insufficient_confident_cells_and_excessive_lethal_fraction"] += 1
            elif rejected_for_confidence:
                rejection_reasons["insufficient_confident_cells"] += 1
            else:
                rejection_reasons["excessive_lethal_fraction"] += 1
            reset_continuity(phase_report)
            continue

        accepted_frame += 1
        phase_report["accepted_frames"] += 1
        for name in features:
            for key in feature_lethal_cells[name]:
                prior = feature_runs[name].get(key)
                if prior is not None and prior["last_frame"] == accepted_frame - 1:
                    feature_run = {"frames": prior["frames"] + 1, "start_ns": prior["start_ns"]}
                else:
                    feature_run = {"frames": 1, "start_ns": timestamp}
                feature_run["last_frame"] = accepted_frame
                feature_runs[name][key] = feature_run
                feature_duration = max(0.0, (timestamp - feature_run["start_ns"]) / 1e9)
                if feature_run["frames"] > max_run[name]["frames"] or feature_duration > max_run[name]["duration_s"]:
                    max_run[name] = {"frames": feature_run["frames"], "duration_s": feature_duration}
        for key in frame_lethal_cells:
            prior = runs.get(key)
            if prior is not None and prior["last_frame"] == accepted_frame - 1:
                run = {"frames": prior["frames"] + 1, "start_ns": prior["start_ns"]}
            else:
                run = {"frames": 1, "start_ns": timestamp}
            run["last_frame"] = accepted_frame
            runs[key] = run
            duration = max(0.0, (timestamp - run["start_ns"]) / 1e9)
            if run["frames"] > combined_max_run["frames"] or duration > combined_max_run["duration_s"]:
                combined_max_run = {"frames": run["frames"], "duration_s": duration}
            phase_runs = phase_report["runs"]
            phase_prior = phase_runs.get(key)
            if phase_prior is not None and phase_prior["last_frame"] == phase_report["accepted_frames"] - 1:
                phase_run = {"frames": phase_prior["frames"] + 1, "start_ns": phase_prior["start_ns"]}
            else:
                phase_run = {"frames": 1, "start_ns": timestamp}
            phase_run["last_frame"] = phase_report["accepted_frames"]
            phase_runs[key] = phase_run
            phase_duration = max(0.0, (timestamp - phase_run["start_ns"]) / 1e9)
            if (phase_run["frames"] > phase_report["max_run"]["frames"] or
                    phase_duration > phase_report["max_run"]["duration_s"]):
                phase_report["max_run"] = {"frames": phase_run["frames"], "duration_s": phase_duration}
    report = {
        "topic": args.topic,
        "frames": total_frames,
        "unique_timestamp_frames": unique_frame,
        "duration_s": None if first_ns is None else (last_ns - first_ns) / 1e9,
        "confidence": {
            "min_observations": args.min_observations,
            "max_observation_age_frames": args.max_observation_age_frames,
            "min_confident_cells": args.min_confident_cells,
            "max_lethal_fraction": args.max_lethal_fraction,
            "missing_confidence_frames": missing_confidence_frames,
            "repeated_or_rollback_frames": repeated_or_rollback_frames,
            "accepted_frames": accepted_frame,
            "rejected_frames": rejected_frames,
            "rejection_reasons": dict(sorted(rejection_reasons.items())),
            "low_confidence_cells": low_confidence_cells,
            "confident_cells_p50": percentile(confident_cells_per_frame, 0.50),
            "confident_cells_p95": percentile(confident_cells_per_frame, 0.95),
            "confident_cells_p99": percentile(confident_cells_per_frame, 0.99),
        },
        "features": {},
        "combined_lethal": {
            "semantics": "any_feature_per_world_cell",
            "max_contiguous_frames": combined_max_run["frames"],
            "max_contiguous_duration_s": combined_max_run["duration_s"],
        },
        "phases": {},
    }
    for name in features:
        report["features"][name] = {
            "p50": percentile(values[name], 0.50),
            "p95": percentile(values[name], 0.95),
            "p99": percentile(values[name], 0.99),
            "lethal_threshold": {
                "slope": args.slope_lethal,
                "step_residual": args.residual_lethal,
                "roughness": args.roughness_lethal,
            }[name],
            "lethal_cell_frame_count": lethal_cells[name],
            "max_contiguous_frames": max_run[name]["frames"],
            "max_contiguous_duration_s": max_run[name]["duration_s"],
        }
    for name, phase_report in phase_reports.items():
        report["phases"][name] = {
            "frames": phase_report["frames"],
            "accepted_frames": phase_report["accepted_frames"],
            "rejected_frames": phase_report["rejected_frames"],
            "confident_cells_p50": percentile(phase_report["confident_cells"], 0.50),
            "confident_cells_p95": percentile(phase_report["confident_cells"], 0.95),
            "lethal_cell_frame_count": phase_report["lethal_cell_frame_count"],
            "max_contiguous_frames": phase_report["max_run"]["frames"],
            "max_contiguous_duration_s": phase_report["max_run"]["duration_s"],
            "features": {
                feature: {
                    "p50": percentile(phase_report["values"][feature], 0.50),
                    "p95": percentile(phase_report["values"][feature], 0.95),
                    "p99": percentile(phase_report["values"][feature], 0.99),
                } for feature in features
            },
        }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
