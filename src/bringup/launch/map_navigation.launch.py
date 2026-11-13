import json
import os
import re
import subprocess

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml as _RewrittenYaml


class RewrittenYaml(_RewrittenYaml):
    """Rewrite array elements and literal dotted ROS parameter names on Humble."""

    def updateYamlPathVals(self, data, keys, value):
        literal_key = ".".join(keys)
        if isinstance(data, dict) and literal_key in data:
            data[literal_key] = value
            return data
        key = int(keys[0]) if isinstance(data, list) else keys[0]
        if len(keys) == 1:
            data[key] = value
        else:
            data[key] = self.updateYamlPathVals(data[key], keys[1:], value)
        return data


def inspect_chassis_can_interface(output, returncode):
    lines = output.strip().splitlines()
    first_line = lines[0] if lines else ""
    flag_text = ""
    if "<" in first_line and ">" in first_line:
        flag_text = first_line.split("<", 1)[1].split(">", 1)[0]
    is_up = "UP" in flag_text.split(",")
    bitrate_configured = re.search(r"\bbitrate\s+\d+\b", output) is not None
    state_match = re.search(
        r"^\s*can(?:\s+<[^>]+>)?\s+state\s+([A-Z][A-Z-]*)\b",
        output,
        flags=re.IGNORECASE | re.MULTILINE,
    )
    can_state = state_match.group(1).upper() if state_match else "UNKNOWN"
    ready = returncode == 0 and is_up and bitrate_configured and can_state == "ERROR-ACTIVE"
    return ready, can_state


def validate_chassis_can_interface(context):
    enabled = LaunchConfiguration("enable_chassis").perform(context).lower()
    if enabled not in {"true", "1", "yes", "on"}:
        return []

    interface = LaunchConfiguration("can_interface").perform(context)
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", "dev", interface],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as error:
        raise RuntimeError(f"Cannot inspect chassis CAN interface {interface}: {error}") from error

    output = result.stdout.strip()
    ready, can_state = inspect_chassis_can_interface(output, result.returncode)
    if not ready:
        raise RuntimeError(
            f"Chassis CAN interface {interface} is not ready "
            f"(controller state {can_state}; interface must be UP with its chassis bitrate "
            "configured and controller state ERROR-ACTIVE). "
            "Configure the interface for the attached controller and restart map navigation, "
            "or use enable_chassis:=false when motion is not required."
        )
    return []


def configure_chassis_model(context):
    model = LaunchConfiguration("chassis_model").perform(context)
    if model == "diff":
        model = "two_wheel"
    config_path = os.path.join(
        get_package_share_directory("bringup"), "config", "chassis_models.yaml")
    with open(config_path, "r", encoding="utf-8") as model_file:
        models = yaml.safe_load(model_file)
    if model not in models:
        raise RuntimeError("chassis_model must be two_wheel or omni")
    profile = models[model]
    return [
        SetLaunchConfiguration("chassis_model", model),
        SetLaunchConfiguration("robot_footprint", json.dumps(profile["footprint"])),
        SetLaunchConfiguration("robot_footprint_flat", json.dumps(profile["flat_footprint"])),
        LogInfo(msg=(
            f"Chassis model={model} footprint={profile['length']:.3f}x"
            f"{profile['width']:.3f} m profile={config_path}")),
    ]


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    terrain_map_file = LaunchConfiguration("terrain_map_file")
    ground_source_file = LaunchConfiguration("ground_source_file")
    occupancy_source_file = LaunchConfiguration("occupancy_source_file")
    saved_map_root = LaunchConfiguration("saved_map_root")
    map_source_id = LaunchConfiguration("map_source_id")
    display_pcd_file = LaunchConfiguration("display_pcd_file")
    display_ground_cache_file = LaunchConfiguration("display_ground_cache_file")
    display_structure_cache_file = LaunchConfiguration("display_structure_cache_file")
    display_max_height_above_ground = LaunchConfiguration("display_max_height_above_ground")
    local_cloud_radius = LaunchConfiguration("local_cloud_radius")
    local_ground_voxel_size = LaunchConfiguration("local_ground_voxel_size")
    local_cloud_max_height_above_ground = LaunchConfiguration(
        "local_cloud_max_height_above_ground")
    local_cloud_timeout = LaunchConfiguration("local_cloud_timeout")
    global_inflation_alpha = LaunchConfiguration("global_inflation_alpha")
    diagnostic_cell_enabled = LaunchConfiguration("diagnostic_cell_enabled")
    diagnostic_cell_x = LaunchConfiguration("diagnostic_cell_x")
    diagnostic_cell_y = LaunchConfiguration("diagnostic_cell_y")
    enable_global_terrain = LaunchConfiguration("enable_global_terrain")
    enable_local_terrain = LaunchConfiguration("enable_local_terrain")
    sensor_height = LaunchConfiguration("sensor_height")
    ground_min_range = LaunchConfiguration("ground_min_range")
    ground_max_range = LaunchConfiguration("ground_max_range")
    ground_residual_xy_tolerance = LaunchConfiguration("ground_residual_xy_tolerance")
    ground_residual_z_tolerance = LaunchConfiguration("ground_residual_z_tolerance")
    footprint_clearing_enabled = LaunchConfiguration("footprint_clearing_enabled")
    visualize = LaunchConfiguration("visualize")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    allow_unknown = LaunchConfiguration("allow_unknown")
    enable_frontier_exploration = LaunchConfiguration("enable_frontier_exploration")
    enable_fastdem = LaunchConfiguration("enable_fastdem")
    require_terrain = LaunchConfiguration("require_terrain")
    robot_height = LaunchConfiguration("robot_height")
    clearance_margin = LaunchConfiguration("clearance_margin")
    enable_chassis = LaunchConfiguration("enable_chassis")
    chassis_model = LaunchConfiguration("chassis_model")
    robot_footprint = LaunchConfiguration("robot_footprint")
    robot_footprint_flat = LaunchConfiguration("robot_footprint_flat")
    omni_lateral_speed = PythonExpression([
        "'0.15' if '", chassis_model, "' == 'omni' else '0.0'"])
    omni_lateral_accel = PythonExpression([
        "'0.30' if '", chassis_model, "' == 'omni' else '0.0'"])
    can_interface = LaunchConfiguration("can_interface")
    body_to_base_x = LaunchConfiguration("body_to_base_x")
    body_to_base_y = LaunchConfiguration("body_to_base_y")
    body_to_base_yaw = LaunchConfiguration("body_to_base_yaw")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    lifecycle_request_timeout = LaunchConfiguration("lifecycle_request_timeout")
    lifecycle_state_poll_period = LaunchConfiguration("lifecycle_state_poll_period")
    localization_status_timeout = LaunchConfiguration("localization_status_timeout")
    fastlio_health_loss_grace = LaunchConfiguration("fastlio_health_loss_grace")
    startup_epoch_ns = LaunchConfiguration("startup_epoch_ns")
    localization_ready_topic = LaunchConfiguration("localization_ready_topic")
    navigation_paused_topic = LaunchConfiguration("navigation_paused_topic")
    navigation_pause_request_topic = LaunchConfiguration("navigation_pause_request_topic")
    navigation_pause_status_topic = LaunchConfiguration("navigation_pause_status_topic")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_timeout = LaunchConfiguration("initial_pose_timeout")

    patchworkpp_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        output="screen",
        parameters=[{
            "base_frame": "base_link",
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "sensor_height": ParameterValue(sensor_height, value_type=float),
            "num_iter": 3,
            "num_lpr": 20,
            "num_min_pts": 0,
            "th_seeds": 0.20,
            "th_dist": 0.06,
            "th_seeds_v": 0.25,
            "th_dist_v": 0.9,
            "max_range": ParameterValue(ground_max_range, value_type=float),
            "min_range": ParameterValue(ground_min_range, value_type=float),
            "ground_residual_xy_tolerance": ParameterValue(
                ground_residual_xy_tolerance, value_type=float),
            "ground_residual_z_tolerance": ParameterValue(
                ground_residual_z_tolerance, value_type=float),
            "uprightness_thr": 0.707,
            "verbose": True,
        }],
        remappings=[("pointcloud_topic", "/cloud_registered_body")],
    )

    fastdem_node = Node(
        package="fastdem_ros2",
        executable="fastdem_node",
        name="fastdem",
        output="screen",
        condition=IfCondition(enable_fastdem),
        parameters=[{
            "config_file": PathJoinSubstitution(
                [FindPackageShare("bringup"), "config", "fastdem_local.yaml"]),
            "input_scan": "/patchworkpp/ground",
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    nonground_height_filter = Node(
        package="bringup",
        executable="nonground_height_filter",
        name="nonground_height_filter",
        output="screen",
        parameters=[{
            "input_topic": "/patchworkpp/nonground",
            "ground_topic": "/fastdem/mapping/gridmap",
            "blocking_topic": "/obstacles/blocking",
            "global_blocking_topic": "/obstacles/global_blocking",
            "global_clearing_topic": "/obstacles/global_clearing",
            "ground_veto_topic": "/obstacles/ground_veto",
            "ground_veto_fence_service": "/obstacles/ground_veto_fence",
            "overhead_topic": "/obstacles/overhead",
            "map_timeout": 1.5,
            "temporal_obstacle_filter_enabled": True,
            "tracking_frame": "odom",
            "raytrace_topic": "/cloud_registered_body",
            "obstacle_grid_resolution": 0.10,
            "obstacle_confirmation_window_frames": 5,
            "obstacle_confirmation_hits": 2,
            "obstacle_clear_ray_frames": 2,
            "obstacle_max_unobserved_frames": 20,
            "obstacle_local_window_size": 6.0,
            "obstacle_raytrace_height_tolerance": 0.25,
            "obstacle_raytrace_min_range": 0.1,
            "obstacle_raytrace_max_range": 5.0,
            "global_obstacle_max_unobserved_frames": 10,
            "global_obstacle_local_window_size": 20.0,
            "global_obstacle_max_range": 10.0,
            "global_obstacle_raytrace_max_range": 10.0,
            "robot_height": ParameterValue(robot_height, value_type=float),
            "clearance_margin": ParameterValue(clearance_margin, value_type=float),
            "sensor_height": ParameterValue(sensor_height, value_type=float),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        # Keep the PCD prior separate from the live, fused /map.
        remappings=[("map", "/map_prior")],
    )

    global_map_fusion = Node(
        package="bringup",
        executable="global_map_fusion_node",
        name="global_map_fusion",
        output="screen",
        parameters=[{
            "prior_map_topic": "/map_prior",
            "map_topic": "/map",
            "updates_topic": "/map_updates",
            "ground_topic": "/patchworkpp/ground",
            "global_clearing_topic": "/obstacles/global_clearing",
            "ground_veto_topic": "/obstacles/ground_veto",
            "ground_veto_fence_service": "/obstacles/ground_veto_fence",
            "clearing_ray_topic": "/cloud_registered_body",
            "health_topic": "/fastlio/health",
            "free_observations": 3,
            "clear_observations": 5,
            "online_obstacle_clear_observations": 5,
            "raytrace_min_range": 0.1,
            "raytrace_max_range": 20.0,
            "max_update_range": 20.0,
            "max_map_extension_m": 20.0,
            "max_map_cells": 4000000,
            "health_timeout": 0.50,
            "tf_lookup_timeout": 0.15,
            "full_map_publish_period": 1.0,
            "terrain_map_file": terrain_map_file,
            "ground_provenance_file": ground_source_file,
            "occupancy_source_file": occupancy_source_file,
            "footprint_clearing_enabled": ParameterValue(
                footprint_clearing_enabled, value_type=bool),
            "diagnostic_cell_enabled": ParameterValue(
                diagnostic_cell_enabled, value_type=bool),
            "diagnostic_cell_x": ParameterValue(diagnostic_cell_x, value_type=float),
            "diagnostic_cell_y": ParameterValue(diagnostic_cell_y, value_type=float),
            "base_frame": "base_link",
            "robot_footprint": ParameterValue(robot_footprint_flat),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    map_snapshot_saver = Node(
        package="bringup",
        executable="map_snapshot_saver_node",
        name="map_snapshot_saver",
        output="screen",
        parameters=[{
            "output_root": saved_map_root,
            "source_id": map_source_id,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    frontier_explorer = Node(
        package="bringup",
        executable="frontier_explorer_node",
        name="frontier_explorer",
        output="screen",
        condition=IfCondition(enable_frontier_exploration),
        parameters=[{
            "map_topic": "/map",
            "map_updates_topic": "/map_updates",
            "costmap_topic": "/global_costmap/costmap",
            "prior_map_topic": "/map_prior",
            "base_frame": "base_link",
            "action_name": "navigate_to_pose",
            "navigation_ready_topic": "/navigation_ready",
            "health_topic": "/fastlio/health",
            "observation_topic": "/obstacles/blocking",
            "terrain_topic": "/fastdem/postprocess/gridmap",
            "require_terrain": ParameterValue(enable_fastdem, value_type=bool),
            "terrain_frame": "odom",
            "terrain_max_age": 1.5,
            "terrain_max_future_stamp_offset": 0.1,
            "terrain_min_observations": 3,
            "terrain_max_observation_age_frames": 3,
            "terrain_min_confident_cells": 20,
            "terrain_max_lethal_fraction": 0.90,
            "terrain_slope_lethal_threshold": 35.0,
            "terrain_residual_step_lethal_threshold": 0.25,
            "terrain_roughness_lethal_threshold": 0.25,
            "robot_footprint": ParameterValue(robot_footprint_flat),
            "min_frontier_cells": 4,
            "search_period": 2.0,
            "tf_timeout": 0.10,
            "exclusion_radius": 0.60,
            "max_map_extension_m": 20.0,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    static_terrain_map_publisher = Node(
        package="bringup",
        executable="static_terrain_map_publisher",
        name="static_terrain_map_publisher",
        output="screen",
        condition=IfCondition(visualize),
        parameters=[{
            "terrain_map_file": terrain_map_file,
            "display_pcd_file": display_pcd_file,
            "display_ground_cache_file": display_ground_cache_file,
            "display_structure_cache_file": display_structure_cache_file,
            "ground_display_voxel_size": 0.12,
            "display_max_height_above_ground": ParameterValue(
                display_max_height_above_ground, value_type=float),
            "local_cloud_topic": "/cloud_registered_body",
            "local_ground_display_topic": "/navigation_display/local_ground_cloud",
            "local_structure_display_topic": "/navigation_display/local_structure_cloud",
            "local_ground_voxel_size": ParameterValue(local_ground_voxel_size, value_type=float),
            "local_cloud_radius": ParameterValue(local_cloud_radius, value_type=float),
            "local_cloud_max_height_above_ground": ParameterValue(
                local_cloud_max_height_above_ground, value_type=float),
            "local_cloud_fallback_ground_height": ParameterValue(
                sensor_height, value_type=float),
            "clearance_ray_observations": 3,
            "clearance_ray_min_range": 0.5,
            "clearance_ray_max_range": 20.0,
            "clearance_ray_height_resolution": 0.10,
            "diagnostic_cell_enabled": ParameterValue(
                diagnostic_cell_enabled, value_type=bool),
            "diagnostic_cell_x": ParameterValue(diagnostic_cell_x, value_type=float),
            "diagnostic_cell_y": ParameterValue(diagnostic_cell_y, value_type=float),
            "local_cloud_timeout": ParameterValue(local_cloud_timeout, value_type=float),
            "global_inflation_alpha": ParameterValue(global_inflation_alpha, value_type=float),
            "topic": "/terrain_map/static",
            "frame_id": "map",
            "merge_live_terrain": False,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("bringup"), "launch", "nav2_navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": RewrittenYaml(
                source_file=params_file,
                param_rewrites={
                    "planner_server.ros__parameters.GridBased.allow_unknown": allow_unknown,
                    "local_costmap.local_costmap.ros__parameters.terrain_layer.enabled":
                        enable_local_terrain,
                    "global_costmap.global_costmap.ros__parameters.static_terrain_layer.enabled":
                        enable_global_terrain,
                    "global_costmap.global_costmap.ros__parameters.static_terrain_layer.robot_height":
                        robot_height,
                    "global_costmap.global_costmap.ros__parameters.static_terrain_layer.clearance_margin":
                        clearance_margin,
                    "global_costmap.global_costmap.ros__parameters.static_terrain_layer.terrain_map_file":
                        terrain_map_file,
                    "global_costmap.global_costmap.ros__parameters.footprint": robot_footprint,
                    "local_costmap.local_costmap.ros__parameters.footprint": robot_footprint,
                    "controller_server.ros__parameters.FollowPath.footprint_model.vertices":
                        robot_footprint,
                    "controller_server.ros__parameters.FollowPath.max_vel_y":
                        omni_lateral_speed,
                    "controller_server.ros__parameters.FollowPath.acc_lim_y":
                        omni_lateral_accel,
                    "velocity_smoother.ros__parameters.max_velocity.1": omni_lateral_speed,
                    "velocity_smoother.ros__parameters.min_velocity.1": PythonExpression([
                        "'-0.15' if '", chassis_model, "' == 'omni' else '0.0'"]),
                    "velocity_smoother.ros__parameters.max_accel.1": omni_lateral_accel,
                    "velocity_smoother.ros__parameters.max_decel.1": PythonExpression([
                        "'-0.30' if '", chassis_model, "' == 'omni' else '0.0'"]),
                    "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius":
                        PythonExpression([
                            "'0.55' if '", chassis_model, "' == 'omni' else '0.25'"]),
                },
                convert_types=True,
            ),
            "autostart": "false",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    chassis_node = Node(
        package="bringup",
        executable="chassis_driver",
        name="chassis_driver",
        output="screen",
        condition=IfCondition(enable_chassis),
        parameters=[{
            "chassis_model": chassis_model,
            "cmd_vel_topic": "/cmd_vel",
            "can_interface": can_interface,
            "can_id": 80,
            "max_raw_speed": 800,
            "max_linear_speed": 0.5,
            "max_angular_speed": 1.8,
            "send_rate": 200.0,
            "command_timeout": 0.5,
            "require_perception": True,
            "perception_timeout": 1.5,
            "require_relocalization": False,
            "require_navigation_ready": True,
            "navigation_ready_topic": "/navigation_ready",
            "startup_epoch_ns": ParameterValue(startup_epoch_ns, value_type=int),
            "require_terrain": ParameterValue(enable_fastdem, value_type=bool),
            "terrain_topic": "/fastdem/postprocess/gridmap",
            "terrain_frame": "odom",
            "terrain_max_age": 1.5,
            "terrain_max_future_stamp_offset": 0.1,
            "terrain_min_observations": 3,
            "terrain_max_observation_age_frames": 3,
            "terrain_min_confident_cells": 20,
            "terrain_max_lethal_fraction": 0.90,
            "terrain_slope_lethal_threshold": 35.0,
            "terrain_residual_step_lethal_threshold": 0.25,
            "terrain_roughness_lethal_threshold": 0.25,
            "require_reverse_costmap": True,
            "require_motion_costmap": True,
            "motion_costmap_topic": "/local_costmap/costmap_raw",
            "reverse_costmap_topic": "/local_costmap/costmap_raw",
            "reverse_costmap_timeout": 0.75,
            "reverse_odometry_timeout": 0.4,
            "reverse_base_frame": "base_link",
            "reverse_projection_time": 0.5,
            "reverse_linear_deceleration": 0.5,
            "reverse_angular_deceleration": 0.6,
            "reverse_safety_margin": 0.15,
            "reverse_motion_threshold": 0.05,
            "reverse_confirmation_samples": 3,
            "reverse_latch_reset_cycles": 20,
            "reverse_unknown_is_blocked": True,
            "reverse_block_cost": 253,
            "motion_footprint": ParameterValue(robot_footprint_flat),
            "reverse_footprint": ParameterValue(robot_footprint_flat),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config, "-f", "map"],
        condition=IfCondition(visualize),
    )

    path_display_node = Node(
        package="bringup",
        executable="nav2_path_display_node",
        name="nav2_path_display",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "navigation_goal_status_topic": "/navigate_to_pose/_action/status",
            "execution_goal_status_topic": "/navigate_to_pose_internal/_action/status",
            "requested_goal_topic": "/navigation/requested_goal",
            "goal_identity_topic": "/navigation/goal_identity",
        }],
    )

    navigation_goal_proxy_node = Node(
        package="bringup",
        executable="navigation_goal_proxy_node",
        name="navigation_goal_proxy",
        output="screen",
        parameters=[{
            "public_action": "/navigate_to_pose",
            "internal_action": "/navigate_to_pose_internal",
            "navigation_ready_topic": "/navigation_ready",
            "global_frame": "map",
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    navigation_gate_node = Node(
        package="bringup",
        executable="navigation_gate_node",
        name="navigation_gate",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            "use_relocalization_status": True,
            "localization_ready_topic": localization_ready_topic,
            "navigation_paused_topic": navigation_paused_topic,
            "navigation_pause_request_topic": navigation_pause_request_topic,
            "navigation_pause_status_topic": navigation_pause_status_topic,
            "localization_status_timeout": ParameterValue(
                localization_status_timeout, value_type=float),
            "lifecycle_service": "/lifecycle_manager_navigation/manage_nodes",
            "navigation_ready_topic": "/navigation_ready",
            "require_fastlio_health": True,
            "fastlio_health_topic": "/fastlio/health",
            "fastlio_health_timeout": 0.5,
            "require_perception": True,
            "perception_topic": "/obstacles/ground_veto",
            "perception_timeout": 1.5,
            "require_terrain_status": ParameterValue(require_terrain, value_type=bool),
            "terrain_status_topic": "/fastdem/terrain_valid",
            "terrain_timeout": 1.2,
            "terrain_valid_topic": "/navigation/terrain_valid",
            "fastlio_health_loss_grace": ParameterValue(
                fastlio_health_loss_grace, value_type=float),
            "lifecycle_request_timeout": ParameterValue(
                lifecycle_request_timeout, value_type=float),
            "lifecycle_state_poll_period": ParameterValue(
                lifecycle_state_poll_period, value_type=float),
            "managed_nodes": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    initial_pose_publisher = Node(
        package="bringup",
        executable="publish_initial_pose.py",
        name="initial_pose_publisher",
        output="screen",
        condition=IfCondition(publish_initial_pose),
        parameters=[{
            "x": ParameterValue(initial_pose_x, value_type=float),
            "y": ParameterValue(initial_pose_y, value_type=float),
            "yaw": ParameterValue(initial_pose_yaw, value_type=float),
            "timeout_sec": ParameterValue(initial_pose_timeout, value_type=float),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", description="PCD 转换得到的 Nav2 地图 YAML"),
        DeclareLaunchArgument(
            "pcd_file", default_value="",
            description="兼容旧脚本参数；重定位阶段由 start_map_navigation.sh 单独启动",
        ),
        DeclareLaunchArgument(
            "terrain_map_file",
            description="离线静态 2.5D 地形地图 NPZ 的绝对路径",
        ),
        DeclareLaunchArgument(
            "ground_source_file", default_value="",
            description="可选的匹配导航快照地面来源 PGM",
        ),
        DeclareLaunchArgument(
            "occupancy_source_file", default_value="",
            description="可选的匹配导航快照占据来源 PGM",
        ),
        DeclareLaunchArgument(
            "saved_map_root",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), ".ros", "nav2_saved_maps"]),
            description="融合导航地图的版本化保存根目录，与 PCD 转换缓存分开",
        ),
        DeclareLaunchArgument(
            "map_source_id", default_value="unspecified",
            description="原始 PCD 内容标识，用于隔离和恢复匹配的导航地图",
        ),
        DeclareLaunchArgument(
            "display_pcd_file", default_value="",
            description="可选的降采样静态 PCD 调试显示缓存路径；默认画面不加载",
        ),
        DeclareLaunchArgument(
            "display_ground_cache_file", default_value="",
            description="可选的预分类静态 PCD 地面显示缓存",
        ),
        DeclareLaunchArgument(
            "display_structure_cache_file", default_value="",
            description="可选的预分类、预索引静态 PCD 结构缓存",
        ),
        DeclareLaunchArgument(
            "display_max_height_above_ground", default_value="1.2",
            description="显示 PCD 相对局部地面最大高度（米），用于裁掉天花板",
        ),
        DeclareLaunchArgument(
            "local_cloud_radius", default_value="5.0",
            description="RViz 中实时局部点云的机器人中心半径（米）",
        ),
        DeclareLaunchArgument(
            "local_ground_voxel_size", default_value="0.12",
            description="实时局部地面显示的 XY 采样尺寸（米）",
        ),
        DeclareLaunchArgument(
            "local_cloud_max_height_above_ground", default_value="1.2",
            description="实时局部点云相对地面的显示高度上限（米）",
        ),
        DeclareLaunchArgument(
            "local_cloud_timeout", default_value="1.0",
            description="实时局部点云超过此时间未更新时清空 RViz 显示（秒）",
        ),
        DeclareLaunchArgument(
            "global_inflation_alpha", default_value="0.25",
            description="全局膨胀层 RViz 透明度（0 到 1）",
        ),
        DeclareLaunchArgument(
            "diagnostic_cell_enabled", default_value="false",
            description="启用指定 map 坐标格的地面证据、障碍否决和显示时序诊断",
        ),
        DeclareLaunchArgument(
            "diagnostic_cell_x", default_value="0.0",
            description="诊断目标格中心的 map X 坐标（米）",
        ),
        DeclareLaunchArgument(
            "diagnostic_cell_y", default_value="0.0",
            description="诊断目标格中心的 map Y 坐标（米）",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("bringup"), "config", "nav2_map_navigation.yaml"])),
        DeclareLaunchArgument(
            "sensor_height", default_value="0.25",
            description="base_link 原点离地高度（米），请按实车测量覆盖",
        ),
        DeclareLaunchArgument(
            "ground_min_range",
            default_value="0.25",
            description="Patchwork++ 地面估计最小水平距离（米），与 FastDEM 输入范围一致",
        ),
        DeclareLaunchArgument("ground_max_range", default_value="80.0"),
        DeclareLaunchArgument("ground_residual_xy_tolerance", default_value="0.10"),
        DeclareLaunchArgument("ground_residual_z_tolerance", default_value="0.03"),
        DeclareLaunchArgument("footprint_clearing_enabled", default_value="true"),
        DeclareLaunchArgument(
            "visualize", default_value="true",
            description="默认启动地图导航 RViz；无显示环境时显式传 visualize:=false",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("bringup"), "rviz", "map_navigation_terrain.rviz"]),
            description="地图导航 RViz 配置",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("robot_height", default_value="1.20"),
        DeclareLaunchArgument("clearance_margin", default_value="0.10"),
        DeclareLaunchArgument(
            "lifecycle_request_timeout", default_value="10.0",
            description="Nav2 lifecycle manager 请求的启动/查询等待时间（秒）",
        ),
        DeclareLaunchArgument("lifecycle_state_poll_period", default_value="0.5"),
        DeclareLaunchArgument(
            "startup_epoch_ns", default_value="0",
            description="start_map_navigation.sh 启动纪元，用于测量总启动至 Nav2 ready 时间",
        ),
        DeclareLaunchArgument(
            "localization_status_timeout", default_value="1.0",
            description="GICP 定位状态心跳超过此时间后暂停 Nav2（秒）"),
        DeclareLaunchArgument(
            "fastlio_health_loss_grace", default_value="1.25",
            description=(
                "FAST-LIO 健康异常后保留当前 Nav2 目标的宽限时间（秒）；"
                "默认 1.25 s 依据恢复门至少 5 帧且稳定 0.5 s，并参考观测最长 1.03 s "
                "（余量 0.22 s）；不覆盖进一步恶化的雷达/IMU 间隔，底盘仍立即停车"
            ),
        ),
        DeclareLaunchArgument(
            "localization_ready_topic", default_value="/localization/handoff_ready",
            description="handoff TF 已有当前严格验收目标且 GICP 定位有效时为 true",
        ),
        DeclareLaunchArgument(
            "navigation_paused_topic", default_value="/navigation_paused",
            description="Nav2 lifecycle 状态已确认 inactive 时发布 true",
        ),
        DeclareLaunchArgument(
            "navigation_pause_request_topic",
            default_value="/localization/navigation_pause_request",
            description="handoff 绑定定位身份的 Nav2 暂停请求",
        ),
        DeclareLaunchArgument(
            "navigation_pause_status_topic",
            default_value="/localization/navigation_pause_status",
            description="NavigationGate 对当前暂停请求发布的会话确认",
        ),
        DeclareLaunchArgument(
            "publish_initial_pose", default_value="false",
            description="显式启用自动发布 /initialpose；必须同时提供地图坐标",
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_timeout", default_value="30.0"),
        DeclareLaunchArgument(
            "allow_unknown", default_value="false",
            description="未知区域仅允许规划器按有限起步规则离开，不允许作为路线穿行"),
        DeclareLaunchArgument(
            "enable_frontier_exploration", default_value="false",
            description=(
                "自动选择已知空闲侧的前沿并导航；在线地图扩展和 costmap 同步验收前保持关闭"
            )),
        DeclareLaunchArgument("enable_fastdem", default_value="true"),
        DeclareLaunchArgument(
            "require_terrain", default_value="false",
            description=(
                "地形数据过期时是否由 navigation_gate 拒绝导航；默认 false，"
                "先恢复实时性并确认数据周期后再启用"
            ),
        ),
        DeclareLaunchArgument(
            "enable_global_terrain",
            default_value="true",
            description="是否加载离线静态全局 2.5D 地形层",
        ),
        DeclareLaunchArgument(
            "enable_local_terrain",
            default_value="true",
            description="是否启用在线局部 FastDEM 地形代价层；可传 false 关闭",
        ),
        DeclareLaunchArgument("enable_chassis", default_value="true"),
        DeclareLaunchArgument(
            "chassis_model", default_value="two_wheel",
            description="共享车身模型：two_wheel 或 omni"),
        DeclareLaunchArgument("can_interface", default_value="can3"),
        DeclareLaunchArgument("body_to_base_x", default_value="-0.098346"),
        DeclareLaunchArgument("body_to_base_y", default_value="0.004472"),
        DeclareLaunchArgument("body_to_base_yaw", default_value="0.0"),
        DeclareLaunchArgument("body_to_base_roll", default_value=""),
        DeclareLaunchArgument("body_to_base_pitch", default_value=""),
        OpaqueFunction(function=configure_chassis_model),
        OpaqueFunction(function=validate_chassis_can_interface),
        patchworkpp_node,
        fastdem_node,
        nonground_height_filter,
        map_server,
        global_map_fusion,
        map_snapshot_saver,
        frontier_explorer,
        static_terrain_map_publisher,
        map_lifecycle_manager,
        navigation_goal_proxy_node,
        nav2_launch,
        navigation_gate_node,
        initial_pose_publisher,
        chassis_node,
        path_display_node,
        rviz_node,
    ])
