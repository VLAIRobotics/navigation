#!/usr/bin/env python3
"""Run a bounded +/- angular velocity chassis verification."""

import argparse
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class AngularVerifier:
    def __init__(self, command_topic, odometry_topic, command, pulse_seconds,
                 zero_seconds, settle_seconds, timeout_seconds):
        self.node = rclpy.create_node("verify_chassis_angular")
        self.publisher = self.node.create_publisher(Twist, command_topic, 10)
        self.node.create_subscription(
            Odometry, odometry_topic, self._odometry_callback,
            qos_profile_sensor_data)
        self.command = command
        self.pulse_seconds = pulse_seconds
        self.zero_seconds = zero_seconds
        self.settle_seconds = settle_seconds
        self.timeout_seconds = timeout_seconds
        self.samples = []

    def _odometry_callback(self, message):
        self.samples.append((time.monotonic(), message.twist.twist.angular.z))

    def _publish(self, angular_z):
        message = Twist()
        message.angular.z = float(angular_z)
        self.publisher.publish(message)

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(0.0)
                next_publish = now + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_connection(self):
        deadline = time.monotonic() + self.timeout_seconds
        while time.monotonic() < deadline:
            if (self.publisher.get_subscription_count() > 0 and
                    len(self.samples) >= 5):
                return True
            self._publish(0.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def _run_pulse(self, command):
        start = time.monotonic()
        next_publish = start
        end = start + self.pulse_seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(command)
                next_publish = now + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self._publish(0.0)
        zero_end = time.monotonic() + self.zero_seconds
        while time.monotonic() < zero_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        return [
            value for timestamp, value in self.samples
            if start + self.settle_seconds <= timestamp <= end - 0.2
        ]

    @staticmethod
    def _report(command, samples):
        if not samples:
            return {"command": command, "samples": 0, "median": None,
                    "peak": None, "gain": None, "same_sign": False}
        median = statistics.median(samples)
        return {
            "command": command,
            "samples": len(samples),
            "median": median,
            "peak": max(abs(value) for value in samples),
            "gain": abs(median) / abs(command),
            "same_sign": command * median > 0.0,
        }

    def run(self):
        if not self._wait_for_connection():
            raise RuntimeError("/cmd_vel_nav has no velocity_smoother subscriber")

        self._spin_for(2.0)
        positive = self._report(self.command, self._run_pulse(self.command))
        negative = self._report(-self.command, self._run_pulse(-self.command))
        self._spin_for(1.0)
        return positive, negative

    def stop(self):
        for _ in range(8):
            self._publish(0.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def destroy(self):
        self.node.destroy_node()


def print_report(positive, negative, max_peak):
    print("\n角速度验证结果")
    for label, report in (("正向", positive), ("反向", negative)):
        if report["median"] is None:
            print(f"{label}: 没有收到有效 /Odometry 样本")
            continue
        print(
            f"{label}: cmd={report['command']:+.2f}, "
            f"odom_median={report['median']:+.3f}, "
            f"gain={report['gain']:.3f}, peak={report['peak']:.3f}, "
            f"samples={report['samples']}"
        )

    reports = (positive, negative)
    valid = all(report["samples"] >= 5 for report in reports)
    valid = valid and all(report["same_sign"] for report in reports)
    valid = valid and all(
        report["median"] is not None and
        0.25 <= abs(report["median"]) <= 0.35
        for report in reports
    )
    valid = valid and all(
        report["peak"] is not None and report["peak"] <= max_peak
        for report in reports
    )
    if valid:
        print("PASS: /cmd_vel_nav=+/-0.30 rad/s 已正确映射到底盘。")
        return 0
    print("FAIL: 请检查 max_angular_speed、话题连接或 /Odometry。")
    return 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-topic", default="/cmd_vel_nav")
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--command", type=float, default=0.30)
    parser.add_argument("--pulse-seconds", type=float, default=3.5)
    parser.add_argument("--zero-seconds", type=float, default=3.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--timeout-seconds", type=float, default=30.0)
    parser.add_argument("--max-peak", type=float, default=0.50)
    args = parser.parse_args()

    if args.command <= 0.0 or args.pulse_seconds <= args.settle_seconds + 0.2:
        parser.error("command must be positive and pulse must exceed settle time")

    rclpy.init()
    verifier = AngularVerifier(
        args.command_topic, args.odometry_topic, args.command,
        args.pulse_seconds, args.zero_seconds, args.settle_seconds,
        args.timeout_seconds)
    try:
        positive, negative = verifier.run()
        return print_report(positive, negative, args.max_peak)
    except KeyboardInterrupt:
        print("\n已中断验证。")
        return 130
    except Exception as error:  # pylint: disable=broad-except
        print(f"FAIL: {error}", file=sys.stderr)
        return 2
    finally:
        verifier.stop()
        verifier.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
