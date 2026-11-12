#!/usr/bin/env python3

import math
import struct
import threading
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    config = str(Path(__file__).with_name("fastdem_runtime.yaml"))
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            launch_ros.actions.Node(
                package="fastdem_ros2",
                executable="fastdem_node",
                name="fastdem",
                parameters=[{"config_file": config}],
                output="screen",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestFastDemRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("fastdem_runtime_test")
        self.processed = 0
        self.near_field_processed_points = 0
        self.delays = []
        self.raw_widths = []
        self.post_widths = []
        self.post_messages = []
        self.post_gridmaps = []
        self.boundary_actions = []
        self.normal_actions = []
        self.publisher = self.node.create_publisher(
            PointCloud2, "/test/ground", qos_profile_sensor_data
        )
        self.subscriptions = [
            self.node.create_subscription(
                PointCloud2,
                "/fastdem/scan/preprocessed",
                self._on_processed,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                PointCloud2, "/fastdem/mapping/cloud", self._on_raw, 10
            ),
            self.node.create_subscription(
                PointCloud2, "/fastdem/postprocess/cloud", self._on_post, 10
            ),
            self.node.create_subscription(
                GridMap, "/fastdem/postprocess/gridmap", self._on_post_gridmap, 10
            ),
            self.node.create_subscription(
                Marker, "/fastdem/mapping/boundary", self._on_boundary, 10
            ),
            self.node.create_subscription(
                MarkerArray, "/fastdem/postprocess/normals", self._on_normals, 10
            ),
        ]
        self.reset = self.node.create_client(Trigger, "/fastdem/reset_map")
        self.postprocess = self.node.create_client(
            Trigger, "/fastdem/run_postprocess"
        )

    def tearDown(self):
        if self.reset.wait_for_service(timeout_sec=0.5):
            future = self.reset.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            if future.done() and future.result() is not None:
                self._spin_for(0.1)
        self.node.destroy_node()

    def _on_processed(self, message):
        self.processed += 1
        self.near_field_processed_points += self._near_field_point_count(message)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        delay = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        self.delays.append(max(0.0, delay))

    @staticmethod
    def _near_field_point_count(message):
        offsets = {field.name: field.offset for field in message.fields}
        if not all(name in offsets for name in ("x", "y", "z")):
            return 0
        endian = ">" if message.is_bigendian else "<"
        count = 0
        for index in range(message.width * message.height):
            base = index * message.point_step
            xyz = [
                struct.unpack_from(endian + "f", message.data, base + offsets[name])[0]
                for name in ("x", "y", "z")
            ]
            if all(math.isfinite(value) for value in xyz):
                horizontal_range = math.hypot(xyz[0], xyz[1])
                spatial_range = math.sqrt(xyz[0] ** 2 + xyz[1] ** 2 + xyz[2] ** 2)
                if 0.25 <= spatial_range < 0.50 and horizontal_range < 0.50:
                    count += 1
        return count

    def _on_raw(self, message):
        self.raw_widths.append(message.width)

    def _on_post(self, message):
        self.post_widths.append(message.width)
        self.post_messages.append(message)

    def _on_post_gridmap(self, message):
        self.post_gridmaps.append(message)

    def _on_boundary(self, message):
        self.boundary_actions.append(message.action)

    def _on_normals(self, message):
        self.normal_actions.append([marker.action for marker in message.markers])

    def _cloud(self, stamp_offset=0.0, points=None):
        message = PointCloud2()
        message.header.frame_id = "base_link"
        stamp_ns = self.node.get_clock().now().nanoseconds + int(stamp_offset * 1e9)
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.height = 1
        if points is None:
            points = [
                (0.55 + row * 0.1, -0.45 + col * 0.1, 0.0)
                for row in range(10)
                for col in range(10)
                if (row, col) != (5, 5)
            ]
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(struct.pack("<fff", *point) for point in points)
        return message

    def _near_field_cloud(self):
        points = []
        for radius in (0.30, 0.34, 0.38, 0.42, 0.46):
            for index in range(64):
                angle = 2.0 * math.pi * index / 64.0
                points.append((radius * math.cos(angle), radius * math.sin(angle), 0.0))
        return self._cloud(points=points)

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_near_field_ground_points_produce_measured_elevation(self):
        self.assertTrue(self.reset.wait_for_service(timeout_sec=5.0))
        future = self.reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result())
        self.assertTrue(future.result().success)
        self._spin_for(0.2)
        self.post_gridmaps.clear()
        self.near_field_processed_points = 0

        for _ in range(30):
            self.publisher.publish(self._near_field_cloud())
            self._spin_for(0.1)
        self._spin_for(0.3)

        self.assertGreater(self.near_field_processed_points, 0)
        self.assertTrue(self.post_gridmaps)
        measured_elevation = False
        for message in self.post_gridmaps:
            if "elevation" not in message.layers or "n_points" not in message.layers:
                continue
            elevation = message.data[message.layers.index("elevation")].data
            point_count = message.data[message.layers.index("n_points")].data
            if any(
                math.isfinite(height) and math.isfinite(points) and points >= 3.0
                for height, points in zip(elevation, point_count)
            ):
                measured_elevation = True
                break
        self.assertTrue(measured_elevation)

    def test_postprocess_continues_during_concurrent_scan_updates(self):
        self.assertTrue(self.reset.wait_for_service(timeout_sec=5.0))
        reset_future = self.reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, reset_future, timeout_sec=5.0)
        self.assertIsNotNone(reset_future.result())
        self.assertTrue(reset_future.result().success)

        self.publisher.publish(self._cloud())
        self._spin_for(0.2)
        self.assertTrue(self.postprocess.wait_for_service(timeout_sec=5.0))

        stop_publishing = threading.Event()

        def publish_scans():
            while not stop_publishing.is_set():
                self.publisher.publish(self._cloud())
                time.sleep(0.005)

        publisher_thread = threading.Thread(target=publish_scans, daemon=True)
        publisher_thread.start()
        try:
            self._spin_for(0.1)
            output_start = len(self.post_messages)
            future = self.postprocess.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            self.assertIsNotNone(future.result())
            self.assertTrue(future.result().success)
        finally:
            stop_publishing.set()
            publisher_thread.join(timeout=1.0)

        self._spin_for(0.2)
        self.assertTrue(
            any(message.width > 0 for message in self.post_messages[output_start:])
        )

    def test_ten_hertz_processing_and_reset_clear(self):
        self._spin_for(1.0)
        self.assertFalse(any(width > 0 for width in self.raw_widths))
        self.assertFalse(any(width > 0 for width in self.post_widths))

        sent = 30
        for _ in range(sent):
            self.publisher.publish(self._cloud())
            self._spin_for(0.1)
        # Require output during integration, before the input stream stops.
        live_stamps = {
            (message.header.stamp.sec, message.header.stamp.nanosec)
            for message in self.post_gridmaps
            if "slope" in message.layers and "n_points" in message.layers
            and all(
                name in message.layers
                for name in ("_normal_x", "_normal_y", "_normal_z")
            )
        }
        self.assertGreaterEqual(len(live_stamps), 5)
        self._spin_for(0.3)

        self.assertGreaterEqual(self.processed, int(sent * 0.9))
        percentile_95 = sorted(self.delays)[int(len(self.delays) * 0.95) - 1]
        self.assertLess(percentile_95, 0.5)
        self.assertTrue(any(width > 0 for width in self.raw_widths))
        self.assertTrue(any(width > 0 for width in self.post_widths))
        self.assertTrue(self.boundary_actions)
        self.assertTrue(self.normal_actions)
        raw_max = max(self.raw_widths)
        post = max(self.post_messages, key=lambda message: message.width)
        self.assertGreater(post.width, raw_max)
        field_names = {field.name for field in post.fields}
        self.assertIn("elevation", field_names)
        self.assertIn("inpainted_mask", field_names)
        self.assertIn("n_points", field_names)
        self.assertIn("observation_age", field_names)
        self.assertTrue(self.post_gridmaps)
        gridmap_layers = set(self.post_gridmaps[-1].layers)
        self.assertIn("n_points", gridmap_layers)
        self.assertIn("observation_age", gridmap_layers)
        self.assertIn("_normal_x", gridmap_layers)
        self.assertIn("_normal_y", gridmap_layers)
        self.assertIn("_normal_z", gridmap_layers)
        mask_offset = next(
            field.offset for field in post.fields if field.name == "inpainted_mask"
        )
        masks = [
            struct.unpack_from("<f", post.data, index * post.point_step + mask_offset)[0]
            for index in range(post.width)
        ]
        self.assertIn(1.0, masks)

        self.assertTrue(self.reset.wait_for_service(timeout_sec=5.0))
        future = self.reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result())
        self.assertTrue(future.result().success)
        self._spin_for(0.2)
        self.assertEqual(self.raw_widths[-1], 0)
        self.assertEqual(self.post_widths[-1], 0)
        self.assertEqual(self.boundary_actions[-1], Marker.DELETEALL)
        self.assertIn(Marker.DELETEALL, self.normal_actions[-1])
        raw_after_response = len(self.raw_widths)
        post_after_response = len(self.post_widths)
        self._spin_for(0.3)
        self.assertTrue(
            all(width == 0 for width in self.raw_widths[raw_after_response:])
        )
        self.assertTrue(
            all(width == 0 for width in self.post_widths[post_after_response:])
        )

        for _ in range(5):
            self.publisher.publish(self._cloud())
            self._spin_for(0.1)
        self._spin_for(0.3)
        self.assertGreater(self.raw_widths[-1], 0)
        self.assertGreater(self.post_widths[-1], 0)

        processed_before_future = self.processed
        self.publisher.publish(self._cloud(stamp_offset=5.0))
        self._spin_for(0.3)
        self.assertEqual(self.processed, processed_before_future)
        self.assertGreater(self.raw_widths[-1], 0)
        self.assertGreater(self.post_widths[-1], 0)

        raw_stale_index = len(self.raw_widths)
        post_stale_index = len(self.post_widths)
        # Let the 1 s freshness deadline elapse, then allow both timers to
        # publish their clear messages.
        self._spin_for(1.5)
        self.assertGreater(len(self.raw_widths), raw_stale_index)
        self.assertGreater(len(self.post_widths), post_stale_index)
        self.assertGreater(self.raw_widths[-1], 0)
        self.assertEqual(self.post_widths[-1], 0)
