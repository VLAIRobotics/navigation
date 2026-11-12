import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    cloud_topic = LaunchConfiguration("cloud_topic")
    base_frame = LaunchConfiguration("base_frame")
    sensor_height = LaunchConfiguration("sensor_height")
    ground_min_range = LaunchConfiguration("ground_min_range")
    ground_max_range = LaunchConfiguration("ground_max_range")
    ground_visualization_range = LaunchConfiguration("ground_visualization_range")
    ground_residual_xy_tolerance = LaunchConfiguration("ground_residual_xy_tolerance")
    ground_residual_z_tolerance = LaunchConfiguration("ground_residual_z_tolerance")
    visualize = LaunchConfiguration("visualize")
    rviz_config = LaunchConfiguration("rviz_config")
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
    enable_fastdem = LaunchConfiguration("enable_fastdem")
    robot_height = LaunchConfiguration("robot_height")
    clearance_margin = LaunchConfiguration("clearance_margin")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fastlio_path_en = LaunchConfiguration("fastlio_path_en")
    fastlio_map_en = LaunchConfiguration("fastlio_map_en")
    fastlio_scan_publish_en = LaunchConfiguration("fastlio_scan_publish_en")
    fastlio_dense_publish_en = LaunchConfiguration("fastlio_dense_publish_en")
    fastlio_scan_bodyframe_pub_en = LaunchConfiguration("fastlio_scan_bodyframe_pub_en")
    fastlio_pcd_save_en = LaunchConfiguration("fastlio_pcd_save_en")
    point_filter_num = LaunchConfiguration("point_filter_num")
    patchwork_th_seeds = LaunchConfiguration("patchwork_th_seeds")
    patchwork_th_dist = LaunchConfiguration("patchwork_th_dist")
    temporal_obstacle_filter_enabled = LaunchConfiguration("temporal_obstacle_filter_enabled")
    global_costmap_updates_topic = LaunchConfiguration("global_costmap_updates_topic")
    global_obstacle_max_range = LaunchConfiguration("global_obstacle_max_range")
    global_obstacle_raytrace_max_range = LaunchConfiguration(
        "global_obstacle_raytrace_max_range")

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
            ("rviz", "false"),
            ("use_sim_time", use_sim_time),
            ("body_to_base_roll", body_to_base_roll),
            ("body_to_base_pitch", body_to_base_pitch),
            ("path_en", fastlio_path_en),
            ("map_en", fastlio_map_en),
            ("scan_publish_en", fastlio_scan_publish_en),
            ("dense_publish_en", fastlio_dense_publish_en),
            ("scan_bodyframe_pub_en", fastlio_scan_bodyframe_pub_en),
            ("pcd_save_en", fastlio_pcd_save_en),
            ("point_filter_num", point_filter_num),
        ],
    )

    patchworkpp_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", cloud_topic),
        ],
        parameters=[
            {
                "base_frame": base_frame,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "sensor_height": ParameterValue(sensor_height, value_type=float),
                "num_iter": 3,
                "num_lpr": 20,
                "num_min_pts": 0,
                "th_seeds": ParameterValue(patchwork_th_seeds, value_type=float),
                "th_dist": ParameterValue(patchwork_th_dist, value_type=float),
                "th_seeds_v": 0.25,
                "th_dist_v": 0.9,
                "max_range": ParameterValue(ground_max_range, value_type=float),
                "min_range": ParameterValue(ground_min_range, value_type=float),
                "ground_visualization_range": ParameterValue(
                    ground_visualization_range, value_type=float
                ),
                "ground_visualization_publish_stride": 2,
                "ground_visualization_voxel_size": 0.08,
                "ground_residual_xy_tolerance": ParameterValue(
                    ground_residual_xy_tolerance, value_type=float
                ),
                "ground_residual_z_tolerance": ParameterValue(
                    ground_residual_z_tolerance, value_type=float
                ),
                "uprightness_thr": 0.707,
                "verbose": True,
            },
            {"algorithm": "patchworkpp"},
        ],
    )

    fastdem_node = Node(
        package="fastdem_ros2",
        executable="fastdem_node",
        name="fastdem",
        output="screen",
        condition=IfCondition(enable_fastdem),
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("bringup"), "config", "fastdem_local.yaml"]
                ),
                "input_scan": "/patchworkpp/ground",
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    nonground_height_filter = Node(
        package="bringup",
        executable="nonground_height_filter",
        name="nonground_height_filter",
        output="screen",
        condition=IfCondition(enable_fastdem),
        parameters=[{
            "input_topic": "/patchworkpp/nonground",
            "ground_topic": "/fastdem/mapping/gridmap",
            "blocking_topic": "/obstacles/blocking",
            "global_costmap_updates_topic": global_costmap_updates_topic,
            "overhead_topic": "/obstacles/overhead",
            "temporal_obstacle_filter_enabled": ParameterValue(
                temporal_obstacle_filter_enabled, value_type=bool),
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
            "global_obstacle_max_range": ParameterValue(
                global_obstacle_max_range, value_type=float),
            "global_obstacle_raytrace_max_range": ParameterValue(
                global_obstacle_raytrace_max_range, value_type=float),
            "robot_height": ParameterValue(robot_height, value_type=float),
            "clearance_margin": ParameterValue(clearance_margin, value_type=float),
            "sensor_height": ParameterValue(sensor_height, value_type=float),
            "map_timeout": 1.5,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("bringup"), "rviz", rviz_config]
            ),
        ],
        condition=IfCondition(visualize),
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
                "cloud_topic",
                default_value="/cloud_registered_body",
                description="patchworkpp 输入完整去畸变点云话题",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="patchworkpp 输出点云 frame_id（校准后的水平坐标）",
            ),
            DeclareLaunchArgument(
                "sensor_height",
                default_value="0.25",
                description="base_link 原点离地高度（米），请按实测值修改",
            ),
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
                "ground_visualization_range",
                default_value="5.0",
                description="RViz 地面点云窗口半径（米）",
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
                "visualize",
                default_value="true",
                description="是否启动 RViz",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="fastlio_patchwork.rviz",
                description="RViz 配置文件名",
            ),
            DeclareLaunchArgument(
                "window_radius",
                default_value="5.0",
                description="静态地图局部显示窗口半径（米）",
            ),
            DeclareLaunchArgument("body_to_base_x", default_value="-0.098346"),
            DeclareLaunchArgument("body_to_base_y", default_value="0.004472"),
            DeclareLaunchArgument("body_to_base_roll", default_value=""),
            DeclareLaunchArgument("body_to_base_pitch", default_value=""),
            DeclareLaunchArgument("body_to_base_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "enable_fastdem",
                default_value="true",
                description="是否启动 FastDEM 高程节点",
            ),
            DeclareLaunchArgument("robot_height", default_value="0.40"),
            DeclareLaunchArgument("clearance_margin", default_value="0.10"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否让感知和 FastDEM 使用仿真时钟",
            ),
            DeclareLaunchArgument("fastlio_path_en", default_value=""),
            DeclareLaunchArgument("fastlio_map_en", default_value=""),
            DeclareLaunchArgument("fastlio_scan_publish_en", default_value=""),
            DeclareLaunchArgument("fastlio_dense_publish_en", default_value=""),
            DeclareLaunchArgument("fastlio_scan_bodyframe_pub_en", default_value=""),
            DeclareLaunchArgument("point_filter_num", default_value=""),
            DeclareLaunchArgument("patchwork_th_seeds", default_value="0.3"),
            DeclareLaunchArgument("patchwork_th_dist", default_value="0.05"),
            DeclareLaunchArgument("temporal_obstacle_filter_enabled", default_value="false"),
            DeclareLaunchArgument("global_costmap_updates_topic", default_value=""),
            DeclareLaunchArgument("global_obstacle_max_range", default_value="10.0"),
            DeclareLaunchArgument("global_obstacle_raytrace_max_range", default_value="10.0"),
            DeclareLaunchArgument("fastlio_pcd_save_en", default_value=""),
            DeclareLaunchArgument(
                "enable_static_map_filter",
                default_value="false",
                description="是否启动时序体素静态地图过滤器",
            ),
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
            patchworkpp_node,
            fastdem_node,
            nonground_height_filter,
            static_map_filter_launch,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=patchworkpp_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="patchworkpp exited"))],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=fastdem_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="fastdem exited"))],
                )
            ),
            rviz_node,
        ]
    )
