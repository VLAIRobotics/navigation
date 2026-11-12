#!/usr/bin/env python3
import unittest

from analyze_chassis_response import analyze_samples


LINEAR_LEVELS = (0.15, 0.25, 0.35)
ANGULAR_LEVELS = (0.10, 0.20, 0.30)


def pulse(
    commands,
    odometry,
    start,
    linear,
    angular,
    measured_linear,
    measured_angular,
    transient=None,
    coast=None,
    pulse_seconds=3.0,
    zero_seconds=3.0,
    include_zero=True,
):
    """Append one 3 s command pulse and a 3 s zero interval."""
    step = 0.1
    pulse_count = max(1, int(round(pulse_seconds / step)))
    for index in range(pulse_count):
        timestamp = start + index * step
        commands.append((timestamp, linear, angular))
    if include_zero:
        zero_count = max(1, int(round(zero_seconds / step)) + 1)
        for index in range(zero_count):
            timestamp = start + pulse_seconds + index * step
            commands.append((timestamp, 0.0, 0.0))

    # A few settled samples provide one response statistic per pulse.
    for index in (12, 13, 14):
        timestamp = start + index * 0.1
        odometry.append((timestamp, measured_linear, measured_angular))
    if transient is not None:
        odometry.append((start + 0.2, transient[0], transient[1]))
    if coast is None:
        coast = (0.0, 0.0)
    odometry.append((start + pulse_seconds + 0.2, coast[0], coast[1]))
    return start + pulse_seconds + zero_seconds + 0.2


def complete_recording(
    linear_gain=1.0,
    angular_gain=1.5,
    linear_response_sign=1.0,
    angular_response_sign=1.0,
    transient=None,
    coast=None,
    cross_axis=None,
):
    commands, odometry = [], []
    start = 0.0
    for level in LINEAR_LEVELS:
        for sign in (1.0, -1.0):
            for _ in range(3):
                measured_linear = sign * level * linear_gain * linear_response_sign
                measured_angular = 0.0
                if cross_axis is not None:
                    measured_angular = cross_axis
                start = pulse(
                    commands, odometry, start, sign * level, 0.0,
                    measured_linear, measured_angular,
                    transient=transient, coast=coast,
                )
    for level in ANGULAR_LEVELS:
        for sign in (1.0, -1.0):
            for _ in range(3):
                measured_linear = 0.0
                measured_angular = sign * level * angular_gain * angular_response_sign
                if cross_axis is not None:
                    measured_linear = cross_axis
                start = pulse(
                    commands, odometry, start, 0.0, sign * level,
                    measured_linear, measured_angular,
                    transient=transient, coast=coast,
                )
    return commands, odometry


class ChassisResponseAnalysisTest(unittest.TestCase):
    def test_complete_balanced_response_produces_conservative_recommendation(self):
        commands, odometry = complete_recording()
        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertTrue(report["safe_recommendation"])
        self.assertAlmostEqual(report["recommendation"]["max_linear_speed"], 0.5)
        self.assertAlmostEqual(report["recommendation"]["max_angular_speed"], 1.5)
        self.assertEqual(report["pulse_count"], 36)
        self.assertEqual(report["usable_pulse_count"], 36)

    def test_directional_asymmetry_is_rejected(self):
        commands, odometry = complete_recording()
        # Make only the negative linear direction 25% faster.
        for index, (timestamp, linear, angular) in enumerate(odometry):
            if linear < -0.01:
                odometry[index] = (timestamp, linear * 1.25, angular)

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("linear_directional_asymmetry_exceeds_20_percent", report["rejection_reasons"])

    def test_safety_peak_during_settle_window_is_rejected(self):
        commands, odometry = complete_recording(transient=(0.0, 0.60))

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("observed_velocity_exceeds_safety_limit", report["rejection_reasons"])

    def test_safety_peak_after_command_zero_is_rejected(self):
        commands, odometry = complete_recording(coast=(0.60, 0.0))

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("observed_velocity_exceeds_safety_limit", report["rejection_reasons"])

    def test_cross_axis_motion_is_rejected(self):
        commands, odometry = complete_recording(cross_axis=0.06)

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("cross_axis_motion_exceeds_limit", report["rejection_reasons"])

    def test_direction_reversal_is_rejected(self):
        commands, odometry = complete_recording(
            linear_response_sign=-1.0, angular_response_sign=-1.0)

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("response_direction_mismatch", report["rejection_reasons"])

    def test_single_pulse_and_irregular_samples_are_insufficient(self):
        commands, odometry = [], []
        pulse(commands, odometry, 0.0, 0.15, 0.0, 0.15, 0.0)
        pulse(commands, odometry, 6.2, 0.0, 0.10, 0.0, 0.15)

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("missing_linear_pulse_levels", report["rejection_reasons"])
        self.assertIn("missing_angular_pulse_levels", report["rejection_reasons"])

    def test_zero_response_is_rejected_without_division_error(self):
        commands, odometry = complete_recording(linear_gain=0.0, angular_gain=0.0)

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("no_motion_detected", report["rejection_reasons"])
        self.assertIsNone(report["linear_asymmetry_ratio"])
        self.assertIsNone(report["angular_asymmetry_ratio"])

    def test_short_pulse_is_not_counted(self):
        commands, odometry = [], []
        pulse(
            commands, odometry, 0.0, 0.15, 0.0, 0.15, 0.0,
            pulse_seconds=1.0,
        )

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("short_pulse", report["rejection_reasons"])
        self.assertEqual(report["protocol_valid_pulse_count"], 0)

    def test_extra_invalid_pulse_blocks_recommendation(self):
        commands, odometry = complete_recording()
        pulse(
            commands, odometry, commands[-1][0] + 0.2,
            0.15, 0.0, 0.15, 0.0, pulse_seconds=1.0,
        )

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("short_pulse", report["rejection_reasons"])

    def test_missing_zero_interval_and_incomplete_final_pulse_are_rejected(self):
        commands = [(0.0, 0.15, 0.0), (3.0, 0.15, 0.0)]
        odometry = [(1.2, 0.15, 0.0), (2.0, 0.15, 0.0)]

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("missing_zero_interval", report["rejection_reasons"])
        self.assertIn("incomplete_final_pulse", report["rejection_reasons"])

    def test_short_zero_interval_is_rejected(self):
        commands, odometry = [], []
        pulse(
            commands, odometry, 0.0, 0.15, 0.0, 0.15, 0.0,
            zero_seconds=1.0,
        )

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("short_zero_interval", report["rejection_reasons"])

    def test_missing_steady_or_coast_samples_are_rejected(self):
        commands, odometry = [], []
        pulse(
            commands, odometry, 0.0, 0.15, 0.0, 0.15, 0.0,
            coast=(0.0, 0.0),
        )
        # Remove the only settled sample while leaving the zero interval.
        odometry[:] = [sample for sample in odometry if sample[0] > 3.0]

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("insufficient_steady_samples", report["rejection_reasons"])

    def test_missing_coast_samples_are_rejected(self):
        commands, odometry = [], []
        pulse(commands, odometry, 0.0, 0.15, 0.0, 0.15, 0.0)
        odometry[:] = [sample for sample in odometry if sample[0] < 3.0]

        report = analyze_samples(commands, odometry, 0.5, 1.0)

        self.assertFalse(report["safe_recommendation"])
        self.assertIn("insufficient_coast_samples", report["rejection_reasons"])


if __name__ == "__main__":
    unittest.main()
