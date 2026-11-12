from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    body_to_base_x = LaunchConfiguration("body_to_base_x")
    body_to_base_y = LaunchConfiguration("body_to_base_y")
    body_to_base_yaw = LaunchConfiguration("body_to_base_yaw")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    handoff_topic = LaunchConfiguration("handoff_topic")
    handoff_ready_topic = LaunchConfiguration("handoff_ready_topic")
    navigation_pause_request_topic = LaunchConfiguration("navigation_pause_request_topic")
    navigation_pause_status_topic = LaunchConfiguration("navigation_pause_status_topic")
    require_navigation_pause_confirmation = LaunchConfiguration(
        "require_navigation_pause_confirmation")
    handoff_smoothing_time_constant_sec = LaunchConfiguration(
        "handoff_smoothing_time_constant_sec")
    handoff_max_translation_speed_mps = LaunchConfiguration(
        "handoff_max_translation_speed_mps")
    handoff_max_rotation_speed_deg_s = LaunchConfiguration(
        "handoff_max_rotation_speed_deg_s")
    handoff_update_frequency_hz = LaunchConfiguration("handoff_update_frequency_hz")

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("bringup"), "launch", "livox_fastlio_headless.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": config_file,
            "config_path": config_path,
            "body_to_base_roll": body_to_base_roll,
            "body_to_base_pitch": body_to_base_pitch,
            # Navigation must not keep FAST-LIO's disk/path/map publishers
            # alive after relocalization.  Keep the world/body clouds needed by
            # GICP and Patchwork++ respectively.
            "path_en": "false",
            "map_en": "false",
            "scan_publish_en": "true",
            "dense_publish_en": "true",
            "scan_bodyframe_pub_en": "true",
            "pcd_save_en": "false",
            "point_filter_num": "3",
        }.items(),
    )

    body_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", body_to_base_x,
            "--y", body_to_base_y,
            "--z", "0.0",
            "--yaw", body_to_base_yaw,
            "--frame-id", "body",
            "--child-frame-id", "base_link",
        ],
    )

    handoff_node = Node(
        package="bringup",
        executable="map_odom_handoff_node",
        name="map_odom_handoff",
        output="screen",
        parameters=[{
            "handoff_topic": handoff_topic,
            "ready_topic": handoff_ready_topic,
            "navigation_pause_request_topic": navigation_pause_request_topic,
            "navigation_pause_status_topic": navigation_pause_status_topic,
            "require_navigation_pause_confirmation": require_navigation_pause_confirmation,
            "smoothing_time_constant_sec": handoff_smoothing_time_constant_sec,
            "max_translation_speed_mps": handoff_max_translation_speed_mps,
            "max_rotation_speed_deg_s": handoff_max_rotation_speed_deg_s,
            "update_frequency_hz": handoff_update_frequency_hz,
            "use_sim_time": use_sim_time,
        }],
    )
    shutdown_if_handoff_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=handoff_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="map_odom_handoff_node exited; navigation core has no TF authority"
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([FindPackageShare("fast_lio"), "config"])),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("body_to_base_x", default_value="-0.098346"),
        DeclareLaunchArgument("body_to_base_y", default_value="0.004472"),
        DeclareLaunchArgument("body_to_base_yaw", default_value="0.0"),
        DeclareLaunchArgument("body_to_base_roll", default_value=""),
        DeclareLaunchArgument("body_to_base_pitch", default_value=""),
        DeclareLaunchArgument(
            "handoff_topic", default_value="/localization/map_odom_handoff"),
        DeclareLaunchArgument(
            "handoff_ready_topic", default_value="/localization/handoff_ready"),
        DeclareLaunchArgument(
            "navigation_pause_request_topic",
            default_value="/localization/navigation_pause_request"),
        DeclareLaunchArgument(
            "navigation_pause_status_topic",
            default_value="/localization/navigation_pause_status"),
        DeclareLaunchArgument(
            "require_navigation_pause_confirmation", default_value="true"),
        DeclareLaunchArgument("handoff_smoothing_time_constant_sec", default_value="4.0"),
        DeclareLaunchArgument("handoff_max_translation_speed_mps", default_value="0.02"),
        DeclareLaunchArgument("handoff_max_rotation_speed_deg_s", default_value="1.0"),
        DeclareLaunchArgument("handoff_update_frequency_hz", default_value="100.0"),
        shutdown_if_handoff_exits,
        body_to_base_tf,
        handoff_node,
        fastlio_launch,
    ])
