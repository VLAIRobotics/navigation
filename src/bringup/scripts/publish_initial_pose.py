#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from small_gicp_relocalization.msg import RelocalizationStatus


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")
        self.x = float(self.declare_parameter("x", 0.0).value)
        self.y = float(self.declare_parameter("y", 0.0).value)
        self.yaw = float(self.declare_parameter("yaw", 0.0).value)
        self.timeout_sec = max(float(self.declare_parameter("timeout_sec", 30.0).value), 0.1)
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("initial pose x, y and yaw must be finite")

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", QoSProfile(depth=10)
        )
        self.status_subscription = self.create_subscription(
            RelocalizationStatus,
            "/small_gicp/relocalization_status",
            self.status_callback,
            QoSProfile(depth=10),
        )
        self.accepted = False
        self.started_at = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.publish_initial_pose)
        self.get_logger().info(
            "Publishing initial pose until accepted: x=%.3f y=%.3f yaw=%.3f"
            % (self.x, self.y, self.yaw)
        )

    def status_callback(self, message):
        if message.initialization_generation > 0 and not self.accepted:
            self.accepted = True
            self.timer.cancel()
            self.destroy_subscription(self.status_subscription)
            self.status_subscription = None
            self.get_logger().info(
                "GICP accepted initial pose generation=%d; stopped publishing /initialpose"
                % message.initialization_generation
            )

    def publish_initial_pose(self):
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1e-9
        if elapsed >= self.timeout_sec:
            self.timer.cancel()
            self.get_logger().warning(
                "Initial pose publish timeout after %.1fs; navigation remains gated" % elapsed
            )
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
