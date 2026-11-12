from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_fast_lio = LaunchConfiguration("use_fast_lio")
    sample_count = LaunchConfiguration("sample_count")
    timeout_sec = LaunchConfiguration("timeout_sec")
    gyro_threshold = LaunchConfiguration("gyro_threshold")
    publish_tf = LaunchConfiguration("publish_tf")

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
            ("rviz", "false"),
            ("body_to_base_roll", "0.0"),
            ("body_to_base_pitch", "0.0"),
        ],
        condition=IfCondition(use_fast_lio),
    )
    gravity_leveler = Node(
        package="bringup",
        executable="gravity_leveler.py",
        name="gravity_leveler",
        output="screen",
        parameters=[
            {
                "imu_topic": "/livox/imu",
                "body_frame": "body",
                "base_frame": "base_link",
                "sample_count": ParameterValue(sample_count, value_type=int),
                "timeout_sec": ParameterValue(timeout_sec, value_type=float),
                "gyro_threshold": ParameterValue(gyro_threshold, value_type=float),
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value="mid360.yaml"),
            DeclareLaunchArgument(
                "use_fast_lio",
                default_value="false",
                description="是否同时启动 FAST-LIO",
            ),
            DeclareLaunchArgument("sample_count", default_value="200"),
            DeclareLaunchArgument("timeout_sec", default_value="20.0"),
            DeclareLaunchArgument(
                "gyro_threshold",
                default_value="0.05",
                description="静止判定的角速度阈值（rad/s）",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="是否临时发布 body 到 base_link TF",
            ),
            livox_launch,
            fast_lio_launch,
            gravity_leveler,
        ]
    )
