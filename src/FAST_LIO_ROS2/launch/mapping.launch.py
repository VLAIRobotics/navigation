import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    map_file_path = LaunchConfiguration('map_file_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    window_filter = LaunchConfiguration('window_filter')
    window_radius = LaunchConfiguration('window_radius')
    window_world_topic = LaunchConfiguration('window_world_topic')
    body_to_base_roll = LaunchConfiguration('body_to_base_roll')
    body_to_base_pitch = LaunchConfiguration('body_to_base_pitch')
    path_en = LaunchConfiguration('path_en')
    map_en = LaunchConfiguration('map_en')
    scan_publish_en = LaunchConfiguration('scan_publish_en')
    dense_publish_en = LaunchConfiguration('dense_publish_en')
    scan_bodyframe_pub_en = LaunchConfiguration('scan_bodyframe_pub_en')
    pcd_save_en = LaunchConfiguration('pcd_save_en')
    point_filter_num = LaunchConfiguration('point_filter_num')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='FAST-LIO /map_save output PCD path; empty keeps YAML'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_window_filter_cmd = DeclareLaunchArgument(
        'window_filter', default_value='false',
        description='Publish robot-centered sliding window clouds for RViz'
    )
    declare_window_radius_cmd = DeclareLaunchArgument(
        'window_radius', default_value='5.0',
        description='Horizontal radius (m) of the sliding window'
    )
    declare_window_world_topic_cmd = DeclareLaunchArgument(
        'window_world_topic', default_value='/static_map',
        description='World cloud topic used by the sliding window filter'
    )
    declare_body_to_base_roll_cmd = DeclareLaunchArgument(
        'body_to_base_roll', default_value='',
        description='Raw IMU body-to-horizontal-base Roll (rad); empty keeps YAML'
    )
    declare_body_to_base_pitch_cmd = DeclareLaunchArgument(
        'body_to_base_pitch', default_value='',
        description='Raw IMU body-to-horizontal-base Pitch (rad); empty keeps YAML'
    )

    def create_fast_lio_node(context):
        mapping_parameters = {'use_sim_time': use_sim_time}
        if map_file_path.perform(context):
            mapping_parameters['map_file_path'] = map_file_path
        if body_to_base_roll.perform(context):
            mapping_parameters['mapping.body_to_base_roll'] = body_to_base_roll
        if body_to_base_pitch.perform(context):
            mapping_parameters['mapping.body_to_base_pitch'] = body_to_base_pitch
        for parameter_name, launch_value in (
            ('publish.path_en', path_en),
            ('publish.map_en', map_en),
            ('publish.scan_publish_en', scan_publish_en),
            ('publish.dense_publish_en', dense_publish_en),
            ('publish.scan_bodyframe_pub_en', scan_bodyframe_pub_en),
            ('pcd_save.pcd_save_en', pcd_save_en),
            ('point_filter_num', point_filter_num),
        ):
            if launch_value.perform(context):
                mapping_parameters[parameter_name] = launch_value

        fast_lio_node = Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[PathJoinSubstitution([config_path, config_file]),
                        mapping_parameters],
            output='screen'
        )
        return [
            fast_lio_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=fast_lio_node,
                    on_exit=[EmitEvent(
                        event=Shutdown(reason='fastlio_mapping exited'))],
                )
            ),
        ]
    window_filter_node = Node(
        package='fast_lio',
        executable='cloud_window_filter_node',
        parameters=[{
            'window_radius': window_radius,
            'world_cloud_topic': window_world_topic,
        }],
        condition=IfCondition(window_filter),
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_window_filter_cmd)
    ld.add_action(declare_window_radius_cmd)
    ld.add_action(declare_window_world_topic_cmd)
    ld.add_action(declare_body_to_base_roll_cmd)
    ld.add_action(declare_body_to_base_pitch_cmd)
    for name, description in (
        ('path_en', 'Publish FAST-LIO path; empty keeps YAML'),
        ('map_en', 'Publish FAST-LIO map; empty keeps YAML'),
        ('scan_publish_en', 'Publish world scan; empty keeps YAML'),
        ('dense_publish_en', 'Use dense world scan output; empty keeps YAML'),
        ('scan_bodyframe_pub_en', 'Publish body-frame scan; empty keeps YAML'),
        ('pcd_save_en', 'Accumulate complete PCD map; empty keeps YAML'),
        ('point_filter_num', 'Preprocess point sampling interval; empty keeps YAML'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value='', description=description))

    ld.add_action(OpaqueFunction(function=create_fast_lio_node))
    ld.add_action(window_filter_node)
    ld.add_action(rviz_node)

    return ld
