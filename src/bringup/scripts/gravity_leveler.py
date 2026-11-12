#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

GRAVITY_ACCELERATION = 9.80665


def normalize_quaternion(q):
    length = math.sqrt(sum(value * value for value in q))
    if length < 1.0e-9:
        raise RuntimeError("Cannot normalize a zero-length quaternion")
    return tuple(value / length for value in q)


def multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def inverse_quaternion(quaternion):
    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def rotation_from_up_vector(acceleration):
    ax, ay, az = acceleration
    length = math.sqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax / length, ay / length, az / length

    # This rotation maps the measured body acceleration into level coordinates.
    cross_x = ay
    cross_y = -ax
    cross_z = 0.0
    scalar = 1.0 + az
    if scalar < 1.0e-6:
        return (1.0, 0.0, 0.0, 0.0)
    return normalize_quaternion((cross_x, cross_y, cross_z, scalar))


def rotate_vector(quaternion, vector):
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    return multiply_quaternions(
        multiply_quaternions(quaternion, vector_quaternion), inverse
    )[:3]


def quaternion_to_roll_pitch(quaternion):
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
    return roll, pitch


class GravityLeveler(Node):
    def __init__(self):
        super().__init__("gravity_leveler")
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.body_frame = self.declare_parameter("body_frame", "body").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.sample_count = self.declare_parameter("sample_count", 200).value
        self.timeout_sec = self.declare_parameter("timeout_sec", 20.0).value
        self.gyro_threshold = self.declare_parameter("gyro_threshold", 0.05).value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.base_x = self.declare_parameter("base_x", 0.0).value
        self.base_y = self.declare_parameter("base_y", 0.0).value
        self.base_z = self.declare_parameter("base_z", 0.0).value
        self.acceleration_sum = [0.0, 0.0, 0.0]
        self.sample_received = 0
        self.calibration_complete = False
        self.start_time = self.get_clock().now()
        self.broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.1, self.check_calibration)
        self.get_logger().info(
            f"Waiting for {self.sample_count} stationary IMU samples on {self.imu_topic}"
        )

    def imu_callback(self, message):
        if self.calibration_complete:
            return
        gyro = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if not all(math.isfinite(value) for value in gyro):
            raise RuntimeError("IMU gyro contains a non-finite value")
        gyro_norm = math.sqrt(sum(value * value for value in gyro))
        if gyro_norm > self.gyro_threshold:
            raise RuntimeError(
                f"Vehicle moved during calibration: gyro={gyro_norm:.4f} rad/s"
            )
        ax = message.linear_acceleration.x
        ay = message.linear_acceleration.y
        az = message.linear_acceleration.z
        if not all(math.isfinite(value) for value in (ax, ay, az)):
            raise RuntimeError("IMU acceleration contains a non-finite value")
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if not 0.5 <= norm <= 1.5:
            return
        self.acceleration_sum[0] += ax
        self.acceleration_sum[1] += ay
        self.acceleration_sum[2] += az
        self.sample_received += 1

    def check_calibration(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1.0e9
        if self.sample_received >= self.sample_count:
            average = tuple(
                GRAVITY_ACCELERATION * value / self.sample_received
                for value in self.acceleration_sum
            )
            level_quaternion = rotation_from_up_vector(average)
            body_to_base_quaternion = inverse_quaternion(level_quaternion)
            roll, pitch = quaternion_to_roll_pitch(body_to_base_quaternion)
            yaw = 0.0
            body_to_base_rotation = quaternion_from_rpy(roll, pitch, 0.0)
            yaw_rotation = quaternion_from_rpy(0.0, 0.0, yaw)
            body_to_base_quaternion = multiply_quaternions(
                body_to_base_rotation, yaw_rotation
            )
            leveled_accel = rotate_vector(level_quaternion, average)
            self.get_logger().info(f"body_to_base_roll:={roll:.6f}")
            self.get_logger().info(f"body_to_base_pitch:={pitch:.6f}")
            self.get_logger().info(f"body_to_base_yaw:={yaw:.6f}")
            self.get_logger().info(
                "leveled_accel=(%.6f, %.6f, %.6f)"
                % leveled_accel
            )
            self.get_logger().info(
                "body_to_base_quaternion=(%.9f, %.9f, %.9f, %.9f)"
                % body_to_base_quaternion
            )
            if self.publish_tf:
                self.publish_transform(body_to_base_quaternion)
            self.get_logger().info(
                "Gravity calibration complete; temporary body -> base_link TF "
                f"publication={'enabled' if self.publish_tf else 'disabled'}"
            )
            self.calibration_complete = True
            self.timer.cancel()
            return
        if elapsed > self.timeout_sec:
            self.get_logger().error(
                f"Only received {self.sample_received}/{self.sample_count} valid IMU samples"
            )
            raise RuntimeError(
                "IMU gravity calibration timed out: "
                f"received {self.sample_received}/{self.sample_count} samples"
            )

    def publish_transform(self, quaternion):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.body_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.base_x
        transform.transform.translation.y = self.base_y
        transform.transform.translation.z = self.base_z
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GravityLeveler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
