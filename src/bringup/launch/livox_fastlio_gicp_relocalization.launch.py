from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    pcd_file = LaunchConfiguration("pcd_file")
    bbs_global_pose_topic = LaunchConfiguration("bbs_global_pose_topic")
    bbs_seed_retry_sec = LaunchConfiguration("bbs_seed_retry_sec")
    registration_voxel_size = LaunchConfiguration("registration_voxel_size")
    display_voxel_size = LaunchConfiguration("display_voxel_size")
    rviz_config = LaunchConfiguration("rviz_config")
    min_inlier_ratio = LaunchConfiguration("min_inlier_ratio")
    num_threads = LaunchConfiguration("num_threads")
    max_iterations = LaunchConfiguration("max_iterations")
    tracking_frequency_hz = LaunchConfiguration("tracking_frequency_hz")
    tracking_input_voxel_size = LaunchConfiguration("tracking_input_voxel_size")
    tracking_max_iterations = LaunchConfiguration("tracking_max_iterations")
    tracking_max_processing_time_sec = LaunchConfiguration("tracking_max_processing_time_sec")
    tracking_small_correction_translation_m = LaunchConfiguration(
        "tracking_small_correction_translation_m")
    tracking_small_correction_rotation_rad = LaunchConfiguration(
        "tracking_small_correction_rotation_rad")
    tracking_consistency_translation_tolerance_m = LaunchConfiguration(
        "tracking_consistency_translation_tolerance_m")
    tracking_consistency_rotation_tolerance_rad = LaunchConfiguration(
        "tracking_consistency_rotation_tolerance_rad")
    tracking_consistency_frames_required = LaunchConfiguration(
        "tracking_consistency_frames_required")
    tracking_max_translation_correction = LaunchConfiguration(
        "tracking_max_translation_correction")
    tracking_max_rotation_correction = LaunchConfiguration(
        "tracking_max_rotation_correction")
    tracking_max_cloud_age_sec = LaunchConfiguration("tracking_max_cloud_age_sec")
    max_geometry_rmse_m = LaunchConfiguration("max_geometry_rmse_m")
    max_geometry_p95_m = LaunchConfiguration("max_geometry_p95_m")
    stable_frames_required = LaunchConfiguration("stable_frames_required")
    stable_duration = LaunchConfiguration("stable_duration")
    max_stable_translation_spread = LaunchConfiguration("max_stable_translation_spread")
    max_stable_yaw_spread = LaunchConfiguration("max_stable_yaw_spread")
    min_match_sectors = LaunchConfiguration("min_match_sectors")
    require_multistart_consistency = LaunchConfiguration("require_multistart_consistency")
    max_multistart_translation_spread = LaunchConfiguration("max_multistart_translation_spread")
    max_multistart_yaw_spread = LaunchConfiguration("max_multistart_yaw_spread")
    validity_timeout = LaunchConfiguration("validity_timeout")
    max_consecutive_failures = LaunchConfiguration("max_consecutive_failures")
    visualize = LaunchConfiguration("visualize")
    gicp_visualize = LaunchConfiguration("gicp_visualize")
    use_sim_time = LaunchConfiguration("use_sim_time")
    body_to_base_x = LaunchConfiguration("body_to_base_x")
    body_to_base_y = LaunchConfiguration("body_to_base_y")
    body_to_base_yaw = LaunchConfiguration("body_to_base_yaw")
    offline_snapshot_dir = LaunchConfiguration("offline_snapshot_dir")
    timing_log_period_ms = LaunchConfiguration("timing_log_period_ms")
    handoff_topic = LaunchConfiguration("handoff_topic")
    handoff_ready_topic = LaunchConfiguration("handoff_ready_topic")
    enable_handoff = LaunchConfiguration("enable_handoff")
    exit_after_localized = LaunchConfiguration("exit_after_localized")
    enable_sensors = LaunchConfiguration("enable_sensors")
    shutdown_on_exit = LaunchConfiguration("shutdown_on_exit")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_timeout = LaunchConfiguration("initial_pose_timeout")
    publish_body_to_base_tf = LaunchConfiguration("publish_body_to_base_tf")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("livox_ros_driver2"),
                    "launch_ROS2",
                    "msg_MID360_launch.py",
                ]
            )
        ),
        condition=IfCondition(enable_sensors),
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
            ("use_sim_time", use_sim_time),
            ("body_to_base_roll", body_to_base_roll),
            ("body_to_base_pitch", body_to_base_pitch),
        ],
        condition=IfCondition(enable_sensors),
    )

    bbs3d_node = Node(
        package="bbs3d_ros2",
        executable="bbs3d_ros2_node",
        name="bbs3d_ros2_node",
        output="screen",
        parameters=[{
            "config": PathJoinSubstitution([
                FindPackageShare("bbs3d_ros2"),
                "config",
                "bbs3d_relocalization.yaml",
            ]),
            "target_clouds": pcd_file,
            "use_sim_time": use_sim_time,
        }],
    )

    relocalization_node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        name="small_gicp_relocalization",
        output="screen",
        parameters=[{
            "pcd_file": pcd_file,
            "bbs_global_pose_topic": bbs_global_pose_topic,
            "bbs_seed_retry_sec": bbs_seed_retry_sec,
            "registration_voxel_size": registration_voxel_size,
            "display_voxel_size": display_voxel_size,
            "num_threads": num_threads,
            "max_iterations": max_iterations,
            "tracking_frequency_hz": tracking_frequency_hz,
            "tracking_input_voxel_size": tracking_input_voxel_size,
            "tracking_max_iterations": tracking_max_iterations,
            "tracking_max_processing_time_sec": tracking_max_processing_time_sec,
            "tracking_small_correction_translation_m": tracking_small_correction_translation_m,
            "tracking_small_correction_rotation_rad": tracking_small_correction_rotation_rad,
            "tracking_consistency_translation_tolerance_m":
                tracking_consistency_translation_tolerance_m,
            "tracking_consistency_rotation_tolerance_rad":
                tracking_consistency_rotation_tolerance_rad,
            "tracking_consistency_frames_required": tracking_consistency_frames_required,
            "tracking_max_translation_correction": tracking_max_translation_correction,
            "tracking_max_rotation_correction": tracking_max_rotation_correction,
            "tracking_max_cloud_age_sec": tracking_max_cloud_age_sec,
            "min_inlier_ratio": min_inlier_ratio,
            "max_geometry_rmse_m": max_geometry_rmse_m,
            "max_geometry_p95_m": max_geometry_p95_m,
            "stable_frames_required": stable_frames_required,
            "stable_duration": stable_duration,
            "max_stable_translation_spread": max_stable_translation_spread,
            "max_stable_yaw_spread": max_stable_yaw_spread,
            "min_match_sectors": min_match_sectors,
            "require_multistart_consistency": require_multistart_consistency,
            "max_multistart_translation_spread": max_multistart_translation_spread,
            "max_multistart_yaw_spread": max_multistart_yaw_spread,
            "validity_timeout": validity_timeout,
            "max_consecutive_failures": max_consecutive_failures,
            "offline_snapshot_dir": offline_snapshot_dir,
            "offline_snapshot_max_count": LaunchConfiguration("offline_snapshot_max_count"),
            "navigation_map_file": LaunchConfiguration("navigation_map_file"),
            "startup_context": LaunchConfiguration("startup_context"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "manual_seed_input_timeout_sec": LaunchConfiguration(
                "manual_seed_input_timeout_sec"),
            "manual_seed_validation_timeout_sec": LaunchConfiguration(
                "manual_seed_validation_timeout_sec"),
            "manual_seed_bbs_height_max_age_sec": LaunchConfiguration(
                "manual_seed_bbs_height_max_age_sec"),
            "timing_log_period_ms": timing_log_period_ms,
            "handoff_topic": handoff_topic,
            "exit_after_localized": exit_after_localized,
            "use_sim_time": use_sim_time,
        }],
    )

    handoff_node = Node(
        package="bringup",
        executable="map_odom_handoff_node",
        name="map_odom_handoff",
        output="screen",
        condition=IfCondition(enable_handoff),
        parameters=[{
            "handoff_topic": handoff_topic,
            "ready_topic": handoff_ready_topic,
            "use_sim_time": use_sim_time,
        }],
    )
    shutdown_if_handoff_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=handoff_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="map_odom_handoff_node exited; no map -> odom TF authority"
                    )
                )
            ],
        ),
        condition=IfCondition(enable_handoff),
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
        condition=IfCondition(publish_body_to_base_tf),
    )

    initial_pose_publisher = Node(
        package="bringup",
        executable="publish_initial_pose.py",
        name="initial_pose_publisher",
        output="screen",
        condition=IfCondition(publish_initial_pose),
        parameters=[{
            "x": initial_pose_x,
            "y": initial_pose_y,
            "yaw": initial_pose_yaw,
            "timeout_sec": initial_pose_timeout,
            "use_sim_time": use_sim_time,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(gicp_visualize),
    )

    stop_on_relocalization_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=relocalization_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="small_gicp_relocalization_node exited"
                    )
                )
            ],
        ),
        condition=IfCondition(shutdown_on_exit),
    )

    stop_on_bbs3d_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bbs3d_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="bbs3d_ros2_node exited")
                )
            ],
        ),
        condition=IfCondition(shutdown_on_exit),
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
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fast_lio"), "config"]),
                description="FAST-LIO 配置目录",
            ),
            DeclareLaunchArgument("body_to_base_roll", default_value=""),
            DeclareLaunchArgument("body_to_base_pitch", default_value=""),
            DeclareLaunchArgument("body_to_base_x", default_value="-0.098346"),
            DeclareLaunchArgument("body_to_base_y", default_value="0.004472"),
            DeclareLaunchArgument("body_to_base_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "pcd_file",
                description="地图 PCD 的绝对路径",
            ),
            DeclareLaunchArgument(
                "bbs_global_pose_topic",
                default_value="/bbs3d/global_pose",
                description="3D-BBS 全局 map->body 位姿话题",
            ),
            DeclareLaunchArgument(
                "bbs_seed_retry_sec",
                default_value="1.0",
                description="GICP 未通过首个质量门控时允许替换 BBS 初值的间隔",
            ),
            DeclareLaunchArgument(
                "registration_voxel_size",
                default_value="0.20",
                description="GICP 注册地图体素尺寸；不修改原始 PCD",
            ),
            DeclareLaunchArgument(
                "display_voxel_size",
                default_value="0.10",
                description="仅 RViz 静态地图显示的体素尺寸",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("bringup"), "rviz", "gicp_relocalization.rviz"]),
                description="small_gicp 重定位专用 RViz 配置",
            ),
            DeclareLaunchArgument("min_inlier_ratio", default_value="0.35"),
            DeclareLaunchArgument("num_threads", default_value="2",
                                  description="GICP/OpenMP 线程数；6 核板导航默认只占 2 核"),
            DeclareLaunchArgument(
                "max_iterations", default_value="60",
                description="GICP 每帧最大迭代数"),
            DeclareLaunchArgument(
                "tracking_frequency_hz", default_value="1.0",
                description="首次定位验收后的低频 GICP 匹配频率"),
            DeclareLaunchArgument(
                "tracking_input_voxel_size", default_value="0.25",
                description="持续跟踪输入点云的体素尺寸（米）"),
            DeclareLaunchArgument(
                "tracking_max_iterations", default_value="25",
                description="持续跟踪 GICP 每帧最大迭代数"),
            DeclareLaunchArgument(
                "tracking_max_processing_time_sec", default_value="0.8",
                description="持续跟踪配准的最大耗时，超时结果丢弃（秒）"),
            DeclareLaunchArgument(
                "tracking_small_correction_translation_m", default_value="0.05",
                description="免连续确认的持续跟踪平移修正上限（米）"),
            DeclareLaunchArgument(
                "tracking_small_correction_rotation_rad", default_value="0.03",
                description="免连续确认的持续跟踪旋转修正上限（弧度）"),
            DeclareLaunchArgument(
                "tracking_consistency_translation_tolerance_m", default_value="0.10",
                description="较大持续修正连续候选之间的平移容差（米）"),
            DeclareLaunchArgument(
                "tracking_consistency_rotation_tolerance_rad", default_value="0.05",
                description="较大持续修正连续候选之间的旋转容差（弧度）"),
            DeclareLaunchArgument(
                "tracking_consistency_frames_required", default_value="2",
                description="较大持续修正需要通过的连续帧数"),
            DeclareLaunchArgument(
                "tracking_max_translation_correction", default_value="0.50",
                description="持续跟踪候选的最大机器人局部平移修正（米）"),
            DeclareLaunchArgument(
                "tracking_max_rotation_correction", default_value="0.15",
                description="持续跟踪候选的最大旋转修正（弧度）"),
            DeclareLaunchArgument(
                "tracking_max_cloud_age_sec", default_value="1.0",
                description="超过此时间的跟踪点云直接丢弃（秒）"),
            DeclareLaunchArgument("max_geometry_rmse_m", default_value="0.18",
                                  description="最近邻欧氏 RMSE 门限（米，需用三组固定扫描标定）"),
            DeclareLaunchArgument("max_geometry_p95_m", default_value="0.38",
                                  description="最近邻欧氏距离 P95 门限（米，需用三组固定扫描标定）"),
            DeclareLaunchArgument("stable_frames_required", default_value="4"),
            DeclareLaunchArgument("stable_duration", default_value="0.30"),
            DeclareLaunchArgument("max_stable_translation_spread", default_value="0.05"),
            DeclareLaunchArgument("max_stable_yaw_spread", default_value="0.03"),
            DeclareLaunchArgument("min_match_sectors", default_value="5"),
            DeclareLaunchArgument("require_multistart_consistency", default_value="true"),
            DeclareLaunchArgument("max_multistart_translation_spread", default_value="0.15"),
            DeclareLaunchArgument("max_multistart_yaw_spread", default_value="0.05"),
            DeclareLaunchArgument("validity_timeout", default_value="4.0"),
            DeclareLaunchArgument("max_consecutive_failures", default_value="3"),
            DeclareLaunchArgument("offline_snapshot_dir", default_value="",
                                  description="非空时保存有限数量的定位正常/拒绝/降级/丢失样本"),
            DeclareLaunchArgument("offline_snapshot_max_count", default_value="8",
                                  description="快照目录最大样本数，旧样本按序清理"),
            DeclareLaunchArgument("navigation_map_file", default_value="",
                                  description="用于回放的匹配导航 terrain_map.npz"),
            DeclareLaunchArgument("startup_context", default_value="",
                                  description="启动脚本写入快照的地图/底盘/外参上下文"),
            DeclareLaunchArgument("odometry_topic", default_value="/Odometry"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("manual_seed_input_timeout_sec", default_value="3.0",
                                  description="手动定位等待 TF/可用点云的期限（秒）"),
            DeclareLaunchArgument("manual_seed_validation_timeout_sec", default_value="10.0",
                                  description="从首个可用点云开始计算的手动定位验证期限（秒）"),
            DeclareLaunchArgument("manual_seed_bbs_height_max_age_sec", default_value="1.0",
                                  description="冷启动手动定位可借用的 BBS 高度先验最大时间差（秒）"),
            DeclareLaunchArgument("timing_log_period_ms", default_value="1000"),
            DeclareLaunchArgument(
                "handoff_topic", default_value="/localization/map_odom_handoff",
                description="已验收 map->odom 目标话题（transient local）",
            ),
            DeclareLaunchArgument(
                "handoff_ready_topic", default_value="/localization/handoff_ready",
                description="map->odom handoff authority 的 ready 状态话题",
            ),
            DeclareLaunchArgument(
                "enable_handoff", default_value="true",
                description="是否由本 launch 启动带锁的 map->odom authority",
            ),
            DeclareLaunchArgument(
                "exit_after_localized", default_value="false",
                description="LOCALIZED 后发布首个交接并退出 GICP 阶段",
            ),
            DeclareLaunchArgument(
                "enable_sensors", default_value="true",
                description="是否由此 launch 启动 Livox 与 FAST-LIO",
            ),
            DeclareLaunchArgument(
                "shutdown_on_exit", default_value="true",
                description="GICP 退出时是否关闭本 launch（独立重定位默认开启）",
            ),
            DeclareLaunchArgument(
                "publish_initial_pose", default_value="false",
                description="在重定位阶段持续发布一次初始位姿",
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_timeout", default_value="30.0"),
            DeclareLaunchArgument(
                "publish_body_to_base_tf", default_value="true",
                description="是否由本 launch 发布 body->base_link 静态 TF",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="false",
                description="是否启动 RViz（Fixed Frame=map）",
            ),
            DeclareLaunchArgument(
                "gicp_visualize",
                default_value=visualize,
                description="内部 GICP RViz 开关；地图导航可独立关闭此窗口",
            ),
            livox_launch,
            fast_lio_launch,
            body_to_base_tf,
            bbs3d_node,
            initial_pose_publisher,
            shutdown_if_handoff_exits,
            stop_on_bbs3d_exit,
            stop_on_relocalization_exit,
            handoff_node,
            relocalization_node,
            rviz_node,
        ]
    )
