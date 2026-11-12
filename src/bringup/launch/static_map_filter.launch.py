from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable = LaunchConfiguration("enable_static_map_filter")
    parameters = {
        "map_file_path": LaunchConfiguration("static_map_file"),
        "voxel_size": ParameterValue(
            LaunchConfiguration("static_map_voxel_size"), value_type=float),
        "min_observations": ParameterValue(
            LaunchConfiguration("static_map_min_observations"), value_type=int),
        "min_observation_duration": ParameterValue(
            LaunchConfiguration("static_map_min_duration"), value_type=float),
        "candidate_timeout": ParameterValue(
            LaunchConfiguration("static_map_candidate_timeout"), value_type=float),
        "confirmed_miss_limit": ParameterValue(
            LaunchConfiguration("static_map_confirmed_miss_limit"), value_type=int),
        "archive_min_observation_duration": ParameterValue(
            LaunchConfiguration("static_map_archive_min_duration"), value_type=float),
        "archive_min_origin_separation": ParameterValue(
            LaunchConfiguration("static_map_archive_min_origin_separation"), value_type=float),
        "ray_angular_resolution_deg": ParameterValue(
            LaunchConfiguration("static_map_ray_angular_resolution"), value_type=float),
        "min_range": ParameterValue(
            LaunchConfiguration("static_map_min_range"), value_type=float),
        "max_range": ParameterValue(
            LaunchConfiguration("static_map_max_range"), value_type=float),
        "window_radius": ParameterValue(
            LaunchConfiguration("window_radius"), value_type=float),
        "static_map_publish_period": ParameterValue(
            LaunchConfiguration("static_map_publish_period"), value_type=float),
    }

    return LaunchDescription([
        DeclareLaunchArgument("enable_static_map_filter", default_value="false"),
        DeclareLaunchArgument("static_map_file", default_value="clean_map.pcd"),
        DeclareLaunchArgument("static_map_voxel_size", default_value="0.18"),
        DeclareLaunchArgument("static_map_min_observations", default_value="4"),
        DeclareLaunchArgument("static_map_min_duration", default_value="0.4"),
        DeclareLaunchArgument("static_map_candidate_timeout", default_value="1.5"),
        DeclareLaunchArgument(
            "static_map_confirmed_miss_limit", default_value="10",
            description="活动地图动态清理 miss 帧数；约 10 帧=1 秒@10Hz"),
        DeclareLaunchArgument(
            "static_map_archive_min_duration", default_value="1.0",
            description="历史静态体素最少连续观测时间（秒）"),
        DeclareLaunchArgument(
            "static_map_archive_min_origin_separation", default_value="0.5",
            description="历史静态体素至少需要跨越的机器人位移（米）"),
        DeclareLaunchArgument("static_map_ray_angular_resolution", default_value="2.0"),
        DeclareLaunchArgument("static_map_min_range", default_value="0.5"),
        DeclareLaunchArgument("static_map_max_range", default_value="40.0"),
        DeclareLaunchArgument("window_radius", default_value="5.0"),
        DeclareLaunchArgument("static_map_publish_period", default_value="1.0"),
        Node(
            package="bringup",
            executable="static_map_filter_node",
            name="static_map_filter",
            output="screen",
            condition=IfCondition(enable),
            parameters=[parameters],
        ),
    ])
