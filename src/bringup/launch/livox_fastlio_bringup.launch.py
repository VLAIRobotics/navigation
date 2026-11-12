import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    map_file_path = LaunchConfiguration("map_file_path")
    visualize = LaunchConfiguration("visualize")
    window_radius = LaunchConfiguration("window_radius")
    body_to_base_x = LaunchConfiguration("body_to_base_x")
    body_to_base_y = LaunchConfiguration("body_to_base_y")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    body_to_base_yaw = LaunchConfiguration("body_to_base_yaw")
    enable_static_map_filter = LaunchConfiguration("enable_static_map_filter")
    static_map_file = LaunchConfiguration("static_map_file")
    static_map_voxel_size = LaunchConfiguration("static_map_voxel_size")
    static_map_min_observations = LaunchConfiguration("static_map_min_observations")
    static_map_min_duration = LaunchConfiguration("static_map_min_duration")
    static_map_candidate_timeout = LaunchConfiguration("static_map_candidate_timeout")
    static_map_confirmed_miss_limit = LaunchConfiguration("static_map_confirmed_miss_limit")
    static_map_archive_min_duration = LaunchConfiguration("static_map_archive_min_duration")
    static_map_archive_min_origin_separation = LaunchConfiguration("static_map_archive_min_origin_separation")
    static_map_ray_angular_resolution = LaunchConfiguration("static_map_ray_angular_resolution")
    static_map_min_range = LaunchConfiguration("static_map_min_range")
    static_map_max_range = LaunchConfiguration("static_map_max_range")
    static_map_publish_period = LaunchConfiguration("static_map_publish_period")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("livox_ros_driver2"),
                    "launch_ROS2",
                    "msg_MID360_launch.py",
                ]
            )
        )
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments=[
            ("config_path", config_path),
            ("config_file", config_file),
            ("map_file_path", map_file_path),
            ("rviz", visualize),
            ("window_filter", "false"),
            ("window_radius", window_radius),
            ("body_to_base_roll", body_to_base_roll),
            ("body_to_base_pitch", body_to_base_pitch),
        ],
    )

    static_map_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("bringup"), "launch", "static_map_filter.launch.py"]
            )
        ),
        launch_arguments={
            "enable_static_map_filter": enable_static_map_filter,
            "static_map_file": static_map_file,
            "static_map_voxel_size": static_map_voxel_size,
            "static_map_min_observations": static_map_min_observations,
            "static_map_min_duration": static_map_min_duration,
            "static_map_candidate_timeout": static_map_candidate_timeout,
            "static_map_confirmed_miss_limit": static_map_confirmed_miss_limit,
            "static_map_archive_min_duration": static_map_archive_min_duration,
            "static_map_archive_min_origin_separation": static_map_archive_min_origin_separation,
            "static_map_ray_angular_resolution": static_map_ray_angular_resolution,
            "static_map_min_range": static_map_min_range,
            "static_map_max_range": static_map_max_range,
            "window_radius": window_radius,
            "static_map_publish_period": static_map_publish_period,
        }.items(),
    )

    def create_body_to_base_tf(context):
        body_to_base_rotation = quaternion_from_rpy(0.0, 0.0, 0.0)
        yaw_rotation = quaternion_from_rpy(
            0.0, 0.0, float(body_to_base_yaw.perform(context))
        )
        published_rotation = multiply_quaternions(
            body_to_base_rotation, yaw_rotation
        )
        return [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x", body_to_base_x,
                    "--y", body_to_base_y,
                    "--z", "0.0",
                    "--qx", str(published_rotation[0]),
                    "--qy", str(published_rotation[1]),
                    "--qz", str(published_rotation[2]),
                    "--qw", str(published_rotation[3]),
                    "--frame-id", "body",
                    "--child-frame-id", "base_link",
                ],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="mid360.yaml",
                description="FAST-LIO 配置文件，如 mid360.yaml / avia.yaml",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fast_lio"), "config"]
                ),
                description="FAST-LIO 配置目录",
            ),
            DeclareLaunchArgument(
                "map_file_path",
                default_value="",
                description="FAST-LIO /map_save 输出 PCD 的路径；空值使用配置文件设置",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="是否启动 FAST-LIO RViz",
            ),
            DeclareLaunchArgument(
                "window_radius",
                default_value="5.0",
                description="FAST-LIO RViz 滑动窗口半径",
            ),
            DeclareLaunchArgument("body_to_base_x", default_value="-0.098346"),
            DeclareLaunchArgument("body_to_base_y", default_value="0.004472"),
            DeclareLaunchArgument("body_to_base_roll", default_value=""),
            DeclareLaunchArgument("body_to_base_pitch", default_value=""),
            DeclareLaunchArgument("body_to_base_yaw", default_value="0.0"),
            DeclareLaunchArgument("enable_static_map_filter", default_value="false"),
            DeclareLaunchArgument("static_map_file", default_value="clean_map.pcd"),
            DeclareLaunchArgument("static_map_voxel_size", default_value="0.18"),
            DeclareLaunchArgument("static_map_min_observations", default_value="4"),
            DeclareLaunchArgument("static_map_min_duration", default_value="0.4"),
            DeclareLaunchArgument("static_map_candidate_timeout", default_value="1.5"),
            DeclareLaunchArgument("static_map_confirmed_miss_limit", default_value="10"),
            DeclareLaunchArgument("static_map_archive_min_duration", default_value="1.0"),
            DeclareLaunchArgument("static_map_archive_min_origin_separation", default_value="0.5"),
            DeclareLaunchArgument("static_map_ray_angular_resolution", default_value="2.0"),
            DeclareLaunchArgument("static_map_min_range", default_value="0.5"),
            DeclareLaunchArgument("static_map_max_range", default_value="40.0"),
            DeclareLaunchArgument("static_map_publish_period", default_value="1.0"),
            OpaqueFunction(function=create_body_to_base_tf),
            livox_launch,
            fast_lio_launch,
            static_map_filter_launch,
        ]
    )
