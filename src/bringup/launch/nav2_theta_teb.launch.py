import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml as _RewrittenYaml
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    params_file = LaunchConfiguration("params_file")
    sensor_height = LaunchConfiguration("sensor_height")
    ground_min_range = LaunchConfiguration("ground_min_range")
    ground_max_range = LaunchConfiguration("ground_max_range")
    ground_residual_xy_tolerance = LaunchConfiguration("ground_residual_xy_tolerance")
    ground_residual_z_tolerance = LaunchConfiguration("ground_residual_z_tolerance")
    visualize = LaunchConfiguration("visualize")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    body_to_base_x = LaunchConfiguration("body_to_base_x")
    body_to_base_y = LaunchConfiguration("body_to_base_y")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    body_to_base_yaw = LaunchConfiguration("body_to_base_yaw")
    enable_chassis = LaunchConfiguration("enable_chassis")
    enable_fastdem = LaunchConfiguration("enable_fastdem")
    robot_height = LaunchConfiguration("robot_height")
    clearance_margin = LaunchConfiguration("clearance_margin")
    fastlio_scan_publish_en = LaunchConfiguration("fastlio_scan_publish_en")
    point_filter_num = LaunchConfiguration("point_filter_num")
    display_max_height_above_ground = LaunchConfiguration("display_max_height_above_ground")
    display_radius = LaunchConfiguration("display_radius")
    ground_display_voxel_size = LaunchConfiguration("ground_display_voxel_size")
    chassis_model = LaunchConfiguration("chassis_model")
    robot_footprint = LaunchConfiguration("robot_footprint")
    robot_footprint_flat = LaunchConfiguration("robot_footprint_flat")
    omni_lateral_speed = PythonExpression([
        "'0.15' if '", chassis_model, "' == 'omni' else '0.0'"])
    omni_lateral_accel = PythonExpression([
        "'0.30' if '", chassis_model, "' == 'omni' else '0.0'"])
    chassis_params = {
        name: LaunchConfiguration(name)
        for name in (
            "cmd_vel_topic", "can_interface", "can_id", "max_raw_speed",
            "max_linear_speed", "max_angular_speed", "send_rate",
            "command_timeout", "require_perception", "perception_timeout",
            "require_fastlio_health", "fastlio_health_topic",
            "require_relocalization", "relocalization_topic", "relocalization_timeout",
            "terrain_topic", "terrain_frame", "terrain_max_age",
            "terrain_max_future_stamp_offset", "terrain_min_observations",
            "terrain_max_observation_age_frames", "terrain_min_confident_cells",
            "terrain_max_lethal_fraction", "terrain_slope_lethal_threshold",
            "terrain_residual_step_lethal_threshold", "terrain_roughness_lethal_threshold",
        )
    }

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("bringup"),
                    "launch",
                    "livox_fastlio_patchwork.launch.py",
                ]
            )
        ),
        launch_arguments={
            "config_file": config_file,
            "config_path": config_path,
            "sensor_height": sensor_height,
            "ground_min_range": ground_min_range,
            "ground_max_range": ground_max_range,
            "ground_residual_xy_tolerance": ground_residual_xy_tolerance,
            "ground_residual_z_tolerance": ground_residual_z_tolerance,
            "visualize": visualize,
            "rviz_config": rviz_config,
            "body_to_base_x": body_to_base_x,
            "body_to_base_y": body_to_base_y,
            "body_to_base_roll": body_to_base_roll,
            "body_to_base_pitch": body_to_base_pitch,
            "body_to_base_yaw": body_to_base_yaw,
            "enable_fastdem": enable_fastdem,
            "robot_height": robot_height,
            "clearance_margin": clearance_margin,
            "use_sim_time": use_sim_time,
            "fastlio_path_en": "false",
            "fastlio_map_en": "false",
            "fastlio_scan_publish_en": fastlio_scan_publish_en,
            "fastlio_dense_publish_en": "false",
            "fastlio_scan_bodyframe_pub_en": "true",
            "fastlio_pcd_save_en": "false",
            "point_filter_num": point_filter_num,
            "patchwork_th_seeds": "0.20",
            "patchwork_th_dist": "0.06",
            "temporal_obstacle_filter_enabled": "true",
            "global_costmap_updates_topic": "/obstacles/global_costmap_updates",
            # 15 m square rolling costmap reaches 10.61 m at its corners.
            "global_obstacle_max_range": "11.0",
            "global_obstacle_raytrace_max_range": "11.0",
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("bringup"),
                    "launch",
                    "nav2_navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": RewrittenYaml(
                source_file=params_file,
                param_rewrites={
                    "global_costmap.global_costmap.ros__parameters.terrain_layer.enabled":
                        enable_fastdem,
                    "local_costmap.local_costmap.ros__parameters.terrain_layer.enabled":
                        enable_fastdem,
                    "global_costmap.global_costmap.ros__parameters.footprint":
                        robot_footprint,
                    "local_costmap.local_costmap.ros__parameters.footprint":
                        robot_footprint,
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
            # NavigationGate owns lifecycle activation after health checks.
            "autostart": "false",
            "goal_pose_topic": "/navigation/goal_pose_unmanaged",
            "use_composition": "False",
            "use_respawn": "False",
            "navigate_to_pose_action_name": "/navigate_to_pose_internal",
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
            "cmd_vel_topic": chassis_params["cmd_vel_topic"],
            "can_interface": chassis_params["can_interface"],
            "can_id": ParameterValue(chassis_params["can_id"], value_type=int),
            "max_raw_speed": ParameterValue(
                chassis_params["max_raw_speed"], value_type=int),
            "max_linear_speed": ParameterValue(
                chassis_params["max_linear_speed"], value_type=float),
            "max_angular_speed": ParameterValue(
                chassis_params["max_angular_speed"], value_type=float),
            "send_rate": ParameterValue(chassis_params["send_rate"], value_type=float),
            "command_timeout": ParameterValue(
                chassis_params["command_timeout"], value_type=float),
            "require_perception": ParameterValue(
                chassis_params["require_perception"], value_type=bool),
            "perception_timeout": ParameterValue(
                chassis_params["perception_timeout"], value_type=float),
            "require_fastlio_health": ParameterValue(
                chassis_params["require_fastlio_health"], value_type=bool),
            "fastlio_health_topic": chassis_params["fastlio_health_topic"],
            "require_relocalization": ParameterValue(
                chassis_params["require_relocalization"], value_type=bool),
            "relocalization_topic": chassis_params["relocalization_topic"],
            "relocalization_timeout": ParameterValue(
                chassis_params["relocalization_timeout"], value_type=float),
            "require_terrain": ParameterValue(enable_fastdem, value_type=bool),
            "terrain_topic": chassis_params["terrain_topic"],
            "terrain_frame": chassis_params["terrain_frame"],
            "terrain_max_age": ParameterValue(
                chassis_params["terrain_max_age"], value_type=float),
            "terrain_max_future_stamp_offset": ParameterValue(
                chassis_params["terrain_max_future_stamp_offset"], value_type=float),
            "terrain_min_observations": ParameterValue(
                chassis_params["terrain_min_observations"], value_type=int),
            "terrain_max_observation_age_frames": ParameterValue(
                chassis_params["terrain_max_observation_age_frames"], value_type=int),
            "terrain_min_confident_cells": ParameterValue(
                chassis_params["terrain_min_confident_cells"], value_type=int),
            "terrain_max_lethal_fraction": ParameterValue(
                chassis_params["terrain_max_lethal_fraction"], value_type=float),
            "terrain_slope_lethal_threshold": ParameterValue(
                chassis_params["terrain_slope_lethal_threshold"], value_type=float),
            "terrain_residual_step_lethal_threshold": ParameterValue(
                chassis_params["terrain_residual_step_lethal_threshold"], value_type=float),
            "terrain_roughness_lethal_threshold": ParameterValue(
                chassis_params["terrain_roughness_lethal_threshold"], value_type=float),
            "require_motion_costmap": True,
            "motion_costmap_topic": "/local_costmap/costmap_raw",
            "reverse_costmap_timeout": 0.75,
            "reverse_odometry_timeout": 0.25,
            "reverse_base_frame": "base_link",
            "reverse_projection_time": 0.5,
            "reverse_linear_deceleration": 0.5,
            "reverse_angular_deceleration": 0.6,
            "reverse_safety_margin": 0.15,
            "reverse_motion_threshold": 0.05,
            "reverse_unknown_is_blocked": True,
            "reverse_block_cost": 253,
            "motion_footprint": ParameterValue(robot_footprint_flat),
            "reverse_footprint": ParameterValue(robot_footprint_flat),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
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
            "requested_goal_topic": "/navigation/current_waypoint",
            "goal_identity_topic": "/navigation/goal_identity",
        }],
    )

    online_map_fusion_node = Node(
        package="bringup",
        executable="global_map_fusion_node",
        name="global_map_fusion",
        output="screen",
        parameters=[{
            "map_frame": "odom",
            "initialize_without_prior_map": True,
            "initial_map_resolution": 0.10,
            "map_topic": "/map",
            "updates_topic": "/map_updates",
            "ground_topic": "/patchworkpp/ground",
            "global_clearing_topic": "/obstacles/global_clearing",
            "ground_veto_topic": "/obstacles/ground_veto",
            "ground_veto_fence_service": "/obstacles/ground_veto_fence",
            "clearing_ray_topic": "/cloud_registered_body",
            "health_topic": "/fastlio/health",
            "live_terrain_topic": "/fastdem/postprocess/gridmap",
            "require_live_terrain": ParameterValue(enable_fastdem, value_type=bool),
            "live_terrain_timeout": 1.5,
            "live_terrain_max_lag": 0.5,
            "live_terrain_future_tolerance": 0.1,
            "live_terrain_min_points": 3,
            "live_terrain_max_observation_age": 3,
            "ground_max_age": 0.5,
            "ground_future_tolerance": 0.1,
            "free_observations": 3,
            "obstacle_confirmation_frames": 2,
            "fallback_obstacle_confirmation_frames": 5,
            "clear_observations": 5,
            "online_obstacle_clear_observations": 5,
            "raytrace_min_range": 0.1,
            "raytrace_max_range": 20.0,
            "max_update_range": 20.0,
            "max_map_extension_m": 80.0,
            "max_map_cells": 1500000,
            "health_timeout": 0.50,
            "tf_lookup_timeout": 0.15,
            "full_map_publish_period": 1.0,
            "base_frame": "base_link",
            "robot_footprint": ParameterValue(robot_footprint_flat),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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
            "goal_pose_topic": "/goal_pose",
            "global_frame": "odom",
            "manage_goal_routes": True,
            "map_topic": "/map",
            "map_updates_topic": "/map_updates",
            "map_geometry_events_topic": "/map_geometry_events",
            "costmap_topic": "/global_costmap/costmap",
            "costmap_updates_topic": "/global_costmap/costmap_updates",
            "observation_topic": "/cloud_registered_body",
            "base_frame": "base_link",
            "segment_length": 4.0,
            "observation_wait_timeout": 3.0,
            "route_data_wait_timeout": 5.0,
            "route_retry_period": 1.0,
            "max_frontier_attempts": 6,
            "max_no_route_progress_updates": 6,
            "max_route_expansions": 500000,
            "route_goal_tolerance": 0.20,
            "frontier_exclusion_radius": 0.80,
            "start_unknown_escape_radius": 0.50,
            "robot_footprint": ParameterValue(robot_footprint_flat),
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
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "use_relocalization_status": False,
            "localization_ready_topic": "/fastlio/health",
            "lifecycle_service": "/lifecycle_manager_navigation/manage_nodes",
            "navigation_ready_topic": "/navigation_ready",
            "require_fastlio_health": True,
            "fastlio_health_topic": "/fastlio/health",
            "fastlio_health_timeout": 0.5,
            "require_perception": True,
            "perception_topic": "/obstacles/ground_veto",
            "perception_timeout": 1.5,
            "require_terrain_status": ParameterValue(enable_fastdem, value_type=bool),
            "terrain_status_topic": "/fastdem/local_terrain_valid",
            "terrain_timeout": 1.2,
            "terrain_valid_topic": "/navigation/terrain_valid",
            "managed_nodes": [
                "controller_server", "smoother_server", "planner_server",
                "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
            ],
        }],
    )

    navigation_cloud_display_node = Node(
        package="bringup",
        executable="navigation_cloud_display_node",
        name="navigation_cloud_display",
        output="screen",
        parameters=[{
            "input_topic": "/cloud_registered_body",
            "terrain_topic": "/fastdem/postprocess/gridmap",
            "ground_display_topic": "/navigation_display/local_ground_cloud",
            "structure_display_topic": "/navigation_display/local_structure_cloud",
            "history_ground_topic": "/navigation_display/history_ground_cloud",
            "history_structure_topic": "/navigation_display/history_structure_cloud",
            "trajectory_topic": "/navigation_display/traveled_path",
            "odom_frame": "odom",
            "sensor_height": ParameterValue(sensor_height, value_type=float),
            "max_height_above_ground": ParameterValue(
                display_max_height_above_ground, value_type=float),
            "display_radius": ParameterValue(display_radius, value_type=float),
            "ground_voxel_size": ParameterValue(ground_display_voxel_size, value_type=float),
            "structure_voxel_size": 0.12,
            "history_confirmation_frames": 2,
            "fallback_history_confirmation_frames": 5,
            "history_confirmation_window": 0.5,
            "history_max_ground_voxels": 300000,
            "history_max_structure_voxels": 300000,
            "history_candidate_limit": 30000,
            "history_publish_period": 2.0,
            "trajectory_max_poses": 20000,
            "trajectory_min_distance": 0.05,
            "history_health_topic": "/fastlio/health",
            "history_health_timeout": 0.5,
            "terrain_max_age": 1.0,
            "cloud_max_age": 1.0,
            "point_retention_timeout": 1.0,
            "min_terrain_observations": 3,
            "max_terrain_observation_age": 3,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("chassis_model", default_value="two_wheel"),
            DeclareLaunchArgument(
                "config_file",
                default_value="mid360.yaml",
                description="FAST-LIO 配置文件",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fast_lio"), "config"]
                ),
                description="FAST-LIO 配置目录",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("bringup"), "config", "nav2_theta_teb.yaml"]
                ),
                description="Nav2 参数文件",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="no_map_navigation.rviz",
                description="无图导航专用 RViz 配置文件名",
            ),
            DeclareLaunchArgument(
                "display_max_height_above_ground",
                default_value="1.2",
                description="仅显示此高度以下的离地点云（米）",
            ),
            DeclareLaunchArgument(
                "display_radius",
                default_value="5.0",
                description="实时地面/结构点云显示半径（米）",
            ),
            DeclareLaunchArgument(
                "ground_display_voxel_size",
                default_value="0.12",
                description="显示地面点云的 XY 体素尺寸（米）",
            ),
            DeclareLaunchArgument(
                "sensor_height",
                default_value="0.25",
                description="base_link 原点离地高度（米），请按实测值修改",
            ),
            DeclareLaunchArgument("robot_height", default_value="0.40"),
            DeclareLaunchArgument("clearance_margin", default_value="0.10"),
            DeclareLaunchArgument(
                "ground_min_range",
                default_value="0.25",
                description="Patchwork++ 地面估计最小水平距离（米）",
            ),
            DeclareLaunchArgument(
                "ground_max_range",
                default_value="80.0",
                description="Patchwork++ 地面估计最大水平距离（米）",
            ),
            DeclareLaunchArgument(
                "ground_residual_xy_tolerance",
                default_value="0.10",
                description="剔除与局部地面点 XY 距离阈值（米）",
            ),
            DeclareLaunchArgument(
                "ground_residual_z_tolerance",
                default_value="0.03",
                description="剔除低于局部地面且处于噪声范围内的残留高度阈值（米）",
            ),
            DeclareLaunchArgument(
                "point_filter_num",
                default_value="3",
                description="无图导航 FAST-LIO 点云采样步长，与地图导航一致",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="是否启动现有 RViz",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时间",
            ),
            DeclareLaunchArgument(
                "enable_chassis", default_value="true",
                description="是否启动 xArm CAN 底盘驱动",
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("can_interface", default_value="can3"),
            DeclareLaunchArgument("can_id", default_value="80"),
            DeclareLaunchArgument("max_raw_speed", default_value="800"),
            # Physical speed represented by raw +/- max_raw_speed.  These are
            # chassis calibration scales, not Nav2 navigation limits.
            DeclareLaunchArgument("max_linear_speed", default_value="0.5"),
            DeclareLaunchArgument("max_angular_speed", default_value="1.8"),
            DeclareLaunchArgument("send_rate", default_value="200.0"),
            DeclareLaunchArgument("command_timeout", default_value="0.5"),
            DeclareLaunchArgument("require_perception", default_value="true"),
            DeclareLaunchArgument(
                "perception_timeout", default_value="1.5",
                description="感知输入允许的最大间隔（秒）"),
            DeclareLaunchArgument(
                "require_fastlio_health", default_value="true",
                description="FAST-LIO 健康状态无效时停止底盘"),
            DeclareLaunchArgument(
                "fastlio_health_topic", default_value="/fastlio/health"),
            DeclareLaunchArgument(
                "fastlio_scan_publish_en", default_value="false",
                description=(
                    "是否发布 /cloud_registered 世界系点云。无图导航默认关闭以省算力；"
                    "叠加 static_map_filter 或 cloud_window_filter 等消费者时需置 true"),
            ),
            DeclareLaunchArgument(
                "require_relocalization", default_value="false",
                description="是否要求 small_gicp 重定位心跳才能放行底盘"),
            DeclareLaunchArgument(
                "relocalization_topic", default_value="/small_gicp/pose"),
            DeclareLaunchArgument(
                "relocalization_timeout", default_value="1.0",
                description="重定位接受结果的最大间隔（秒）"),
            DeclareLaunchArgument(
                "terrain_topic", default_value="/fastdem/postprocess/gridmap"),
            DeclareLaunchArgument("terrain_frame", default_value="odom"),
            DeclareLaunchArgument("terrain_max_age", default_value="1.5"),
            DeclareLaunchArgument("terrain_max_future_stamp_offset", default_value="0.1"),
            DeclareLaunchArgument("terrain_min_observations", default_value="3"),
            DeclareLaunchArgument("terrain_max_observation_age_frames", default_value="3"),
            DeclareLaunchArgument("terrain_min_confident_cells", default_value="20"),
            DeclareLaunchArgument("terrain_max_lethal_fraction", default_value="0.90"),
            DeclareLaunchArgument("terrain_slope_lethal_threshold", default_value="35.0"),
            DeclareLaunchArgument(
                "terrain_residual_step_lethal_threshold", default_value="0.25"),
            DeclareLaunchArgument(
                "terrain_roughness_lethal_threshold", default_value="0.25"),
            DeclareLaunchArgument(
                "enable_fastdem",
                default_value="true",
                description="导航模式是否启动 FastDEM 地形输入和 terrain_layer",
            ),
            DeclareLaunchArgument("body_to_base_x", default_value="-0.098346"),
            DeclareLaunchArgument("body_to_base_y", default_value="0.004472"),
            DeclareLaunchArgument("body_to_base_roll", default_value=""),
            DeclareLaunchArgument("body_to_base_pitch", default_value=""),
            DeclareLaunchArgument(
                "body_to_base_yaw",
                default_value="0.0",
                description="body 到机器人中心 base_link 的 Yaw（弧度）",
            ),
            OpaqueFunction(function=configure_chassis_model),
            perception_launch,
            online_map_fusion_node,
            nav2_launch,
            navigation_gate_node,
            navigation_goal_proxy_node,
            path_display_node,
            navigation_cloud_display_node,
            chassis_node,
        ]
    )
