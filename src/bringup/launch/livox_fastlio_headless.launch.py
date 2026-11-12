from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    path_en = LaunchConfiguration("path_en")
    map_en = LaunchConfiguration("map_en")
    scan_publish_en = LaunchConfiguration("scan_publish_en")
    dense_publish_en = LaunchConfiguration("dense_publish_en")
    scan_bodyframe_pub_en = LaunchConfiguration("scan_bodyframe_pub_en")
    pcd_save_en = LaunchConfiguration("pcd_save_en")
    point_filter_num = LaunchConfiguration("point_filter_num")
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
            ("config_file", config_file),
            ("config_path", config_path),
            ("rviz", "false"),
            ("body_to_base_roll", body_to_base_roll),
            ("body_to_base_pitch", body_to_base_pitch),
            ("path_en", path_en),
            ("map_en", map_en),
            ("scan_publish_en", scan_publish_en),
            ("dense_publish_en", dense_publish_en),
            ("scan_bodyframe_pub_en", scan_bodyframe_pub_en),
            ("pcd_save_en", pcd_save_en),
            ("point_filter_num", point_filter_num),
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
            "static_map_publish_period": static_map_publish_period,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="mid360.yaml",
                description="FAST-LIO 配置文件，如 mid360.yaml / avia.yaml",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution([FindPackageShare("fast_lio"), "config"]),
            ),
            DeclareLaunchArgument("body_to_base_roll", default_value=""),
            DeclareLaunchArgument("body_to_base_pitch", default_value=""),
            DeclareLaunchArgument("path_en", default_value=""),
            DeclareLaunchArgument("map_en", default_value=""),
            DeclareLaunchArgument("scan_publish_en", default_value=""),
            DeclareLaunchArgument("dense_publish_en", default_value=""),
            DeclareLaunchArgument("scan_bodyframe_pub_en", default_value=""),
            DeclareLaunchArgument("pcd_save_en", default_value=""),
            DeclareLaunchArgument("point_filter_num", default_value=""),
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
            livox_launch,
            fast_lio_launch,
            static_map_filter_launch,
        ]
    )
