import ast
import importlib.util
import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"
RVIZ = Path(__file__).resolve().parents[1] / "rviz"


class NavigationConfigTest(unittest.TestCase):
    def test_navigation_status_panel_has_bringup_include_path(self):
        cmake = (Path(__file__).resolve().parents[1] / "CMakeLists.txt").read_text()
        panel_target = cmake.split("add_library(navigation_status_panel", 1)[1].split(
            "add_library(bringup_goal_cost_condition_bt_node", 1)[0]
        self.assertIn(
            "target_include_directories(navigation_status_panel PRIVATE include)",
            panel_target)

    def test_online_goal_proxy_waits_for_paired_map_geometry(self):
        package = Path(__file__).resolve().parents[1]
        proxy = (package / "src" / "navigation_goal_proxy_node.cpp").read_text()
        launch = (package / "launch" / "nav2_theta_teb.launch.py").read_text()
        self.assertIn('"map_geometry_events_topic": "/map_geometry_events"', launch)
        self.assertIn("online_map_state_.receiveGeometryEvent(*event)", proxy)
        self.assertIn("online_map_state_.synchronizationPending()", proxy)
        self.assertIn("等待在线地图与几何事件配对", proxy)
        self.assertLess(
            proxy.index("online_map_state_.synchronizationPending()"),
            proxy.index("if (state->waiting_for_observation)"))

    def test_frontier_wait_uses_local_gain_and_bounded_exclusion(self):
        package = Path(__file__).resolve().parents[1]
        proxy = (package / "src" / "navigation_goal_proxy_node.cpp").read_text()
        route = (package / "include" / "bringup" / "online_goal_route.hpp").read_text()
        observation_wait = proxy.split(
            "if (state->waiting_for_observation)", 1)[1].split(
                "if (state->last_route_plan_at", 1)[0]
        self.assertIn("unknownCellsAhead", route)
        self.assertIn("frontierObservationGained", route)
        self.assertIn("frontierObservationGained", observation_wait)
        self.assertIn("state->excluded_frontiers.emplace_back", observation_wait)
        self.assertIn("++state->frontier_attempts", observation_wait)
        self.assertIn("state->frontier_unknown_cells_before", proxy)

    def test_grid_map_plugin_definitions_are_target_scoped(self):
        cmake = (Path(__file__).resolve().parents[1] / "CMakeLists.txt").read_text()
        grid_map_core = cmake.index("find_package(grid_map_core REQUIRED)")
        self.assertLess(cmake.index("set(EIGEN_FUNCTORS_PLUGIN "), grid_map_core)
        self.assertLess(cmake.index("set(EIGEN_DENSEBASE_PLUGIN "), grid_map_core)
        self.assertNotIn("add_definitions(", cmake)
        self.assertIn(
            "target_compile_definitions(navigation_cloud_display_node PRIVATE", cmake)
        self.assertIn(
            "target_compile_definitions(test_static_terrain_map_publisher PRIVATE", cmake)

    def test_no_map_navigation_live_cloud_display(self):
        config = yaml.safe_load((RVIZ / "no_map_navigation.rviz").read_text())
        manager = config["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "odom")
        panel_classes = {panel.get("Class") for panel in config["Panels"]}
        self.assertNotIn("nav2_rviz_plugins/Navigation 2", panel_classes)
        self.assertIn("bringup/NavigationStatus", panel_classes)
        tools = {tool.get("Class"): tool for tool in manager["Tools"]}
        self.assertNotIn("nav2_rviz_plugins/GoalTool", tools)
        self.assertEqual(tools["rviz_default_plugins/SetGoal"]["Topic"]["Value"],
                         "/goal_pose")
        displays = {display.get("Name"): display for display in manager["Displays"]}

        for name, topic in (
                ("Live Ground Cloud", "/navigation_display/local_ground_cloud"),
                ("Live Structure Cloud", "/navigation_display/local_structure_cloud")):
            display = displays[name]
            self.assertTrue(display["Enabled"])
            self.assertEqual(display["Topic"]["Value"], topic)
            self.assertEqual(display["Decay Time"], 0.6)
            self.assertEqual(display["Color Transformer"], "RGB8")
        for name, topic in (
                ("Global Path", "/plan_display"),
                ("Local Path", "/local_plan_display"),
                ("Robot Footprint", "/local_costmap/published_footprint")):
            self.assertTrue(displays[name]["Enabled"])
            self.assertEqual(displays[name]["Topic"]["Value"], topic)
        for name in (
                "FastDEM GridMap (debug)", "Global Costmap (debug)",
                "Local Costmap (debug)", "Raw Registered Cloud (debug)"):
            self.assertFalse(displays[name]["Enabled"])

        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "nav2_theta_teb.launch.py").read_text()
        self.assertIn('default_value="no_map_navigation.rviz"', launch)
        self.assertIn('default_value="1.2"', launch)
        self.assertIn('default_value="5.0"', launch)
        self.assertIn('default_value="0.12"', launch)
        self.assertIn('temporal_obstacle_filter_enabled": "true"', launch)
        fastdem_argument = launch.split('"enable_fastdem",', 1)[1].split(")", 1)[0]
        self.assertIn('default_value="true"', fastdem_argument)
        cloud_node = launch.split("navigation_cloud_display_node = Node(", 1)[1].split(
            "    return LaunchDescription(", 1)[0]
        self.assertIn('condition=IfCondition(visualize)', cloud_node)
        self.assertIn('"terrain_topic": "/fastdem/postprocess/gridmap"', cloud_node)
        self.assertIn('"point_retention_timeout": 1.0', cloud_node)

        source = (Path(__file__).resolve().parents[1] / "src" /
                  "navigation_cloud_display_node.cpp").read_text()
        self.assertIn('get_subscription_count() > 0', source)
        self.assertIn('odom_frame_, message->header.frame_id, source_stamp', source)
        self.assertIn('groundElevationAt(', source)
        self.assertIn('height_above_ground > max_height_above_ground_', source)
        self.assertIn('terrain_fallback=%zu', source)
        self.assertIn('cloud_max_age_ || source_age < -future_stamp_tolerance_', source)
        self.assertIn('source_stamp.nanoseconds() <= last_accepted_source_stamp_ns_', source)
        self.assertIn('makeColoredCloud(ground_points, odom_frame_, source_stamp)', source)
        self.assertIn('structure_points, odom_frame_, source_stamp)', source)
        self.assertIn('source_age > cloud_max_age_', source)
        self.assertIn('receive_age > point_retention_timeout_', source)
        self.assertIn('last_display_received_at_', source)
        self.assertNotIn('"/obstacles/', source)

    def test_local_terrain_and_recovery_frames(self):
        for filename in ("nav2_theta_teb.yaml", "nav2_map_navigation.yaml"):
            with self.subTest(filename=filename):
                config = yaml.safe_load((CONFIG / filename).read_text())
                local = config["local_costmap"]["local_costmap"]["ros__parameters"]
                plugins = local["plugins"]
                self.assertIn("terrain_layer", plugins)
                self.assertLess(plugins.index("obstacle_layer"), plugins.index("terrain_layer"))
                self.assertLess(plugins.index("terrain_layer"), plugins.index("inflation_layer"))
                terrain = local["terrain_layer"]
                self.assertEqual(terrain["plugin"], "nav2_terrain_layer::TerrainLayer")
                if filename == "nav2_map_navigation.yaml":
                    self.assertTrue(terrain["residual_step_hard_constraint"])
                self.assertTrue(terrain["enabled"])
                self.assertEqual(terrain["global_frame"], local["global_frame"])
                self.assertEqual(config["behavior_server"]["ros__parameters"]["global_frame"],
                                 local["global_frame"])

    def test_map_navigation_has_static_global_terrain_layer(self):
        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertEqual(
            global_costmap["plugins"],
            ["static_layer", "static_terrain_layer", "inflation_layer"],
        )
        self.assertNotIn("obstacle_layer", global_costmap)
        static_terrain = global_costmap["static_terrain_layer"]
        self.assertEqual(static_terrain["plugin"], "nav2_terrain_layer::StaticTerrainLayer")
        self.assertEqual(static_terrain["global_frame"], "map")
        self.assertTrue(static_terrain["enabled"])
        self.assertTrue(global_costmap["static_layer"]["subscribe_to_updates"])
        self.assertFalse(static_terrain["hard_feature_constraints"])
        self.assertFalse(config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"])

    def test_map_navigation_footprint_clears_recorded_start_lethal_cell(self):
        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        footprint = ast.literal_eval(
            config["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"])
        local = ast.literal_eval(
            config["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
        teb = ast.literal_eval(
            config["controller_server"]["ros__parameters"]["FollowPath"]
            ["footprint_model.vertices"])
        self.assertEqual(footprint, local)
        self.assertEqual(footprint, teb)

        # 2026-10-16 18:07:46 bag: the prior map occupied cell (99,108).
        # Preserve the collision check's 0.25-cell sampling margin.
        start_x, start_y, yaw = -0.0143717, -1.38612, -1.15678
        resolution = 0.10000000149011612
        required_start_escape_radius = max(math.hypot(x, y) for x, y in footprint) + (
            0.25 * resolution)
        planner = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())[
            "planner_server"]["ros__parameters"]["GridBased"]
        self.assertAlmostEqual(
            planner["start_unknown_escape_radius"], required_start_escape_radius, places=4)
        origin_x, origin_y = -10.29036185, -11.85309606
        margin = 0.25 * resolution
        left = origin_x + 99 * resolution - margin
        right = origin_x + 100 * resolution + margin
        bottom = origin_y + 108 * resolution - margin
        top = origin_y + 109 * resolution + margin
        cell = [(left, bottom), (right, bottom), (right, top), (left, top)]

        def intersects(vertices):
            polygon = [
                (start_x + math.cos(yaw) * x - math.sin(yaw) * y,
                 start_y + math.sin(yaw) * x + math.cos(yaw) * y)
                for x, y in vertices
            ]
            axes = [(1.0, 0.0), (0.0, 1.0)] + [
                (polygon[(i + 1) % len(polygon)][1] - polygon[i][1],
                 polygon[i][0] - polygon[(i + 1) % len(polygon)][0])
                for i in range(len(polygon))
            ]
            for axis_x, axis_y in axes:
                footprint_projection = [axis_x * x + axis_y * y for x, y in polygon]
                cell_projection = [axis_x * x + axis_y * y for x, y in cell]
                if (max(footprint_projection) < min(cell_projection) or
                        max(cell_projection) < min(footprint_projection)):
                    return False
            return True

        self.assertTrue(intersects([[x if x >= 0 else -0.40, y] for x, y in footprint]))
        self.assertFalse(intersects(footprint))
        self.assertLessEqual(footprint[2][0], -0.35)

    def test_map_navigation_tightens_residual_step_thresholds(self):
        rolling = yaml.safe_load((CONFIG / "nav2_theta_teb.yaml").read_text())
        mapping = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        reference = rolling["global_costmap"]["global_costmap"]["ros__parameters"]["terrain_layer"]
        rolling_local = rolling["local_costmap"]["local_costmap"]["ros__parameters"]
        rolling_local = rolling_local["terrain_layer"]
        mapping_local = mapping["local_costmap"]["local_costmap"]["ros__parameters"]
        mapping_local = mapping_local["terrain_layer"]
        feature_keys = (
            "slope_soft_threshold", "slope_lethal_threshold",
            "residual_step_soft_threshold", "residual_step_lethal_threshold",
            "roughness_soft_threshold", "roughness_lethal_threshold",
        )
        self.assertEqual({key: rolling_local[key] for key in feature_keys},
                         {key: reference[key] for key in feature_keys})
        map_residual_thresholds = {
            "residual_step_soft_threshold", "residual_step_lethal_threshold",
        }
        for key in feature_keys:
            if key not in map_residual_thresholds:
                self.assertEqual(mapping_local[key], reference[key])
        # 软/致命不再同值：0.03 没有过渡带，3 cm 残差直接成致命候选，通道里会被
        # 撒进几十格 254。软 0.02 先出梯度，致命抬到 0.05 才硬堵。
        self.assertEqual(mapping_local["residual_step_soft_threshold"], 0.02)
        self.assertEqual(mapping_local["residual_step_lethal_threshold"], 0.05)
        self.assertLess(
            mapping_local["residual_step_soft_threshold"],
            mapping_local["residual_step_lethal_threshold"])

    def test_gicp_rviz_shows_dense_map_and_pose_topics(self):
        config = yaml.safe_load((RVIZ / "gicp_relocalization.rviz").read_text())
        manager = config["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        displays = manager["Displays"]
        topics = {
            display.get("Name"): display.get("Topic", {}).get("Value")
            for display in displays
        }
        self.assertEqual(topics["PCD Map"], "/small_gicp/prior_map_display")
        self.assertEqual(topics["Raw FAST-LIO Cloud in Map"], "/cloud_registered")
        raw_display = next(
            display for display in displays if display.get("Name") == "Raw FAST-LIO Cloud in Map")
        self.assertEqual(raw_display["Topic"]["Durability Policy"], "Volatile")
        self.assertEqual(raw_display["Topic"]["Reliability Policy"], "Best Effort")
        self.assertEqual(topics["Aligned Cloud"], "/small_gicp/aligned_cloud")
        self.assertEqual(topics["Localization Preview"], "/small_gicp/pose")
        optional_clouds = {
            "FAST-LIO Body Cloud": "/cloud_registered_body_downsampled",
            "Ground Cloud": "/patchworkpp/ground",
            "Ground Window": "/patchworkpp/ground_window",
            "Non-ground Cloud": "/patchworkpp/nonground",
        }
        for name, topic in optional_clouds.items():
            display = next(item for item in displays if item.get("Name") == name)
            self.assertEqual(display["Topic"]["Value"], topic)
            self.assertFalse(display["Enabled"])
        pcd_display = next(display for display in displays if display.get("Name") == "PCD Map")
        self.assertEqual(pcd_display["Topic"]["Durability Policy"], "Transient Local")

    def test_relocalization_and_navigation_are_separate_stages(self):
        gicp_launch = (Path(__file__).resolve().parents[1] / "launch" /
                       "livox_fastlio_gicp_relocalization.launch.py").read_text()
        map_launch = (Path(__file__).resolve().parents[1] / "launch" /
                      "map_navigation.launch.py").read_text()
        core_launch = (Path(__file__).resolve().parents[1] / "launch" /
                       "navigation_core.launch.py").read_text()
        self.assertNotIn("raw_cloud_topic", gicp_launch)
        self.assertIn('DeclareLaunchArgument("validity_timeout", default_value="10.0")',
                      gicp_launch)
        self.assertIn('"exit_after_localized", default_value="false"', gicp_launch)
        self.assertIn('"handoff_topic", default_value="/localization/map_odom_handoff"',
                      gicp_launch)
        self.assertIn('"use_relocalization_status": True', map_launch)
        self.assertNotIn("livox_fastlio_gicp_relocalization.launch.py", map_launch)
        self.assertIn("map_odom_handoff_node", core_launch)

    def test_fastlio_point_filter_override_is_optional_and_navigation_uses_three(self):
        mapping_launch = (Path(__file__).resolve().parents[2] / "FAST_LIO_ROS2" /
                          "launch" / "mapping.launch.py").read_text()
        headless_launch = (Path(__file__).resolve().parents[1] / "launch" /
                           "livox_fastlio_headless.launch.py").read_text()
        core_launch = (Path(__file__).resolve().parents[1] / "launch" /
                       "navigation_core.launch.py").read_text()
        self.assertIn("('point_filter_num', point_filter_num)", mapping_launch)
        self.assertIn("('point_filter_num', 'Preprocess point sampling interval; empty keeps YAML')",
                      mapping_launch)
        self.assertIn('DeclareLaunchArgument("point_filter_num", default_value="")',
                      headless_launch)
        self.assertIn('("point_filter_num", point_filter_num)', headless_launch)
        self.assertIn('"point_filter_num": "3"', core_launch)
        self.assertIn('"pcd_save_en": "false"', core_launch)

    def test_recovery_unknown_policy_matches_each_navigation_config(self):
        trees = Path(__file__).resolve().parents[1] / "behavior_trees"
        expectations = (
            ("nav2_theta_teb.yaml", "navigate_to_pose_guarded_recovery.xml", "true"),
            ("nav2_map_navigation.yaml",
             "navigate_to_pose_map_guarded_recovery.xml", "false"),
        )
        for filename, tree_name, allow_unknown in expectations:
            with self.subTest(filename=filename):
                config = yaml.safe_load((CONFIG / filename).read_text())
                bt_navigator = config["bt_navigator"]["ros__parameters"]
                self.assertTrue(bt_navigator["default_nav_to_pose_bt_xml"].endswith(tree_name))

                tree = ET.parse(trees / tree_name).getroot()
                recovery = tree.find(
                    ".//ReactiveSequence[@name='RecoveryForNonlethalGoal']")
                if filename == "nav2_map_navigation.yaml":
                    planner = config["planner_server"]["ros__parameters"]["GridBased"]
                    self.assertFalse(planner["allow_unknown"])
                    planning = tree.find(".//GlobalPlanRetry")
                    self.assertIsNotNone(planning)
                    self.assertEqual(
                        [child.tag for child in planning],
                        ["PathCollisionFree", "ComputePathToPose", "PathCollisionFree"])
                    self.assertEqual(planning.get("goal"), "{goal}")
                    self.assertEqual(planning.get("planning_goal"), "{planning_goal}")
                    self.assertEqual(planning.get("candidate_path"), "{candidate_path}")
                    start_gate = planning[0]
                    self.assertEqual(start_gate.get("check_path"), "false")
                    self.assertEqual(start_gate.get("check_goal"), "false")
                    plan = planning[1]
                    self.assertEqual(plan.get("goal"), "{planning_goal}")
                    self.assertEqual(plan.get("path"), "{candidate_path}")
                    candidate_gate = planning[2]
                    self.assertEqual(candidate_gate.get("check_path"), "false")
                    self.assertEqual(candidate_gate.get("check_goal"), "true")
                    self.assertEqual(candidate_gate.get("allow_unknown"), "false")
                    self.assertIsNotNone(tree.find(".//FollowPathRetry"))
                    self.assertFalse(tree.findall(".//UnknownPathGate"))
                    self.assertFalse(tree.findall(".//SignalFollowPathAttempt"))
                    continue

                self.assertIsNotNone(recovery)

                recovery_guards = recovery.findall(".//GoalCostSafe")
                self.assertEqual(len(recovery_guards), 5)
                self.assertTrue(all(
                    guard.get("allow_unknown") == allow_unknown for guard in recovery_guards))

                follow_path = tree.find(".//RecoveryNode[@name='FollowPath']")
                follow_guards = follow_path.findall(".//GoalCostSafe")
                self.assertEqual(len(follow_guards), 2)
                self.assertTrue(all(guard.get("allow_unknown") is None for guard in follow_guards))

        source = (Path(__file__).resolve().parents[1] / "src" /
                  "goal_cost_condition_bt_node.cpp").read_text()
        self.assertIn('BT::InputPort<bool>(\n        "allow_unknown", false', source)
        self.assertIn("Goal is in unknown space:", source)
        self.assertLess(source.index("if (cost == 255)"),
                        source.index("if (static_cast<int>(cost) >= max_cost_exclusive)"))

    def test_map_navigation_uses_retry_control_and_native_follow_path(self):
        trees = Path(__file__).resolve().parents[1] / "behavior_trees"
        tree = ET.parse(trees / "navigate_to_pose_map_guarded_recovery.xml").getroot()
        planning = tree.find(".//GlobalPlanRetry")
        self.assertIsNotNone(planning)
        self.assertEqual([child.tag for child in planning],
                         ["PathCollisionFree", "ComputePathToPose", "PathCollisionFree"])
        self.assertEqual(planning.get("planning_goal"), "{planning_goal}")
        self.assertEqual(planning.get("candidate_path"), "{candidate_path}")
        self.assertEqual(planning[0].get("check_goal"), "false")
        self.assertEqual(planning[1].get("goal"), "{planning_goal}")
        self.assertEqual(planning[1].get("path"), "{candidate_path}")
        self.assertEqual(planning[2].get("goal"), "{goal}")
        self.assertIsNone(planning[2].get("path"))
        self.assertEqual(planning[2].get("check_path"), "false")
        self.assertEqual(planning[2].get("check_goal"), "true")
        self.assertEqual(planning[2].get("allow_unknown"), "false")
        self.assertFalse(tree.findall(".//SmoothPath"))
        retry = tree.find(".//FollowPathRetry")
        self.assertEqual([child.tag for child in retry], ["FollowPath", "PathCollisionFree"])
        self.assertEqual(retry.find("FollowPath").get("path"), "{path}")
        self.assertEqual(retry.find("PathCollisionFree").get("check_goal"), "true")
        self.assertFalse(tree.findall(".//RecoveryNode"))
        self.assertFalse(tree.findall(".//UnknownPathGate"))

        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        navigator = config["bt_navigator"]["ros__parameters"]
        planner = config["planner_server"]["ros__parameters"]["GridBased"]
        controller = config["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(
            controller["reverse_suppression_attempt_topic"],
            "/navigation/follow_path_attempt")
        self.assertEqual(
            controller["follow_path_progress_topic"],
            "/navigation/follow_path_progress")
        self.assertEqual(
            controller["navigation_goal_status_topic"],
            "/navigate_to_pose/_action/status")
        self.assertEqual(controller["reverse_suppression_timeout"], 3.0)
        self.assertAlmostEqual(controller["min_obstacle_dist"], 0.10)
        self.assertAlmostEqual(controller["inflation_dist"], 0.15)
        self.assertAlmostEqual(planner["w_clearance_cost"], 0.50)
        self.assertIn(
            "bringup_follow_path_retry_bt_node",
            navigator["plugin_lib_names"])
        self.assertAlmostEqual(navigator["start_unknown_escape_radius"], 0.4411)
        self.assertEqual(
            planner["start_unknown_escape_radius"], navigator["start_unknown_escape_radius"])
        self.assertFalse(planner["allow_unknown"])
        self.assertTrue(controller["include_costmap_obstacles"])
        self.assertEqual(controller["footprint_model.type"], "polygon")
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertTrue(global_costmap["track_unknown_space"])
        self.assertTrue(global_costmap["static_layer"]["track_unknown_space"])
        self.assertNotIn("obstacle_layer", global_costmap["plugins"])
        self.assertNotIn("obstacle_layer", global_costmap)
        self.assertEqual(
            global_costmap["plugins"],
            ["static_layer", "static_terrain_layer", "inflation_layer"])
        self.assertAlmostEqual(global_costmap["static_terrain_layer"]["robot_height"], 1.20)
        self.assertAlmostEqual(global_costmap["static_terrain_layer"]["clearance_margin"], 0.10)
        self.assertAlmostEqual(global_costmap["footprint_padding"], 0.01)
        self.assertAlmostEqual(global_costmap["inflation_layer"]["inflation_radius"], 0.25)
        self.assertFalse(local_costmap["track_unknown_space"])
        self.assertAlmostEqual(local_costmap["footprint_padding"], 0.01)
        self.assertAlmostEqual(local_costmap["inflation_layer"]["inflation_radius"], 0.25)
        temporal_layer = local_costmap["temporal_obstacle_layer"]
        self.assertIn("temporal_obstacle_layer", local_costmap["plugins"])
        self.assertEqual(temporal_layer["plugin"], "bringup::TemporalObstacleLayer")
        self.assertEqual(temporal_layer["observation_topic"], "/obstacles/blocking")
        self.assertEqual(temporal_layer["point_retention"], 1.0)
        self.assertTrue(local_costmap["obstacle_layer"]["registered_points"]["clearing"])
        self.assertEqual(
            local_costmap["obstacle_layer"]["registered_points"]["raytrace_min_range"], 0.1)

    def test_map_navigation_replanning_costmap_and_inflation_rates(self):
        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertEqual(global_costmap["update_frequency"], 5.0)
        self.assertEqual(global_costmap["publish_frequency"], 5.0)
        controller = config["controller_server"]["ros__parameters"]["FollowPath"]
        # Keep the controller's bounded soft clearance and existing inflation radius.
        self.assertEqual(controller["inflation_dist"], 0.15)
        self.assertEqual(controller["min_obstacle_dist"], 0.10)
        self.assertEqual(controller["max_samples"], 30)
        for costmap in (global_costmap, local_costmap):
            inflation_radius = costmap["inflation_layer"]["inflation_radius"]
            self.assertEqual(inflation_radius, 0.25)

        tree = ET.parse(Path(__file__).resolve().parents[1] / "behavior_trees" /
                        "navigate_to_pose_map_guarded_recovery.xml").getroot()
        self.assertEqual(tree.find(".//GlobalPlanRetry").get("retry_hz"), "1.0")
        self.assertIsNotNone(tree.find(".//FollowPathRetry"))

    def test_global_plan_retry_ports_are_registered_and_bound(self):
        package = Path(__file__).resolve().parents[1]
        tree = ET.parse(package / "behavior_trees" /
                        "navigate_to_pose_map_guarded_recovery.xml").getroot()
        retry = tree.find(".//GlobalPlanRetry")
        self.assertEqual(retry.get("goal"), "{goal}")
        self.assertEqual(retry.get("planning_goal"), "{planning_goal}")
        self.assertEqual(retry.get("path"), "{path}")
        self.assertEqual(retry.get("candidate_path"), "{candidate_path}")
        self.assertEqual(len(retry), 3)
        self.assertEqual(retry[1].tag, "ComputePathToPose")
        self.assertEqual(retry[1].get("goal"), "{planning_goal}")
        self.assertEqual(retry[1].get("path"), "{candidate_path}")

        plugin = (package / "src" / "global_plan_retry_bt_node.cpp").read_text()
        self.assertIn('BT::OutputPort<geometry_msgs::msg::PoseStamped>(\n        "planning_goal"',
                      plugin)
        self.assertIn('BT::BidirectionalPort<nav_msgs::msg::Path>(\n        "candidate_path"',
                      plugin)
        self.assertIn('setOutput("planning_goal", planner_goal_)', plugin)
        self.assertIn('planner_request_id_ = requestId(planner_goal_.header.stamp)', plugin)
        self.assertNotIn('last_planner_request_sequence_', plugin)
        self.assertIn('getInput("candidate_path", candidate)', plugin)
        self.assertIn('writePath("path", candidate)', plugin)
        self.assertIn("category=bt_output_write_failed", plugin)

        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        plugins = config["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
        self.assertIn("bringup_global_plan_retry_bt_node", plugins)

    def test_follow_path_wait_has_no_automatic_20_second_exit(self):
        retry = (Path(__file__).resolve().parents[1] / "src" /
                 "follow_path_retry_bt_node.cpp").read_text()
        controller = (Path(__file__).resolve().parents[1].parents[1] / "nav2" /
                      "teb_local_planner" / "src" / "teb_local_planner_ros.cpp").read_text()
        self.assertIn('"/navigation/follow_path_progress"', retry)
        self.assertNotIn("retry_budget_", retry)
        self.assertNotIn("local_wait_budget_expired", retry)
        self.assertIn("retry_episode_active_", retry)
        self.assertNotIn("samePathGeometry", retry)
        self.assertIn("category=local_wait_released reason=measured_route_progress", retry)
        self.assertIn("global_replan=true teb_diagnostic='%s'", retry)
        failure_recovery = retry.split("haltChild(1);\n    // The controller rejected this attempt.", 1)[1]
        self.assertIn("invalidatePath(target);", failure_recovery)
        self.assertIn("awaiting_fresh_path_ = true;", failure_recovery)
        self.assertIn("publishMeasuredFollowPathProgress", controller)
        self.assertIn("follow_path_progress_pub_->publish(progress)", controller)

        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        for required in (
                '"reverse_motion_threshold": 0.05',
                '"reverse_confirmation_samples": 3',
                '"reverse_latch_reset_cycles": 20',
                '"reverse_unknown_is_blocked": False',
                '"reverse_block_cost": 254',
                '"reverse_safety_margin": 0.15'):
            self.assertIn(required, launch)

    def test_goal_proxy_route_and_frontier_waits_keep_original_goal(self):
        proxy = (Path(__file__).resolve().parents[1] / "src" /
                 "navigation_goal_proxy_node.cpp").read_text()
        wait = proxy.split("RoutePlan waitForRouteProgress(", 1)[1].split(
            "void waitForNextFrontierSearchRound", 1)[0]
        self.assertNotIn("RoutePlanState::ABORT", wait)
        self.assertIn("waiting_for_search_update", proxy)
        self.assertIn("waitForNextFrontierSearchRound(state)", proxy)
        self.assertNotIn("前沿候选全部失败或达到尝试上限", proxy)
        self.assertNotIn("前沿导航失败或达到尝试上限", proxy)

        search_wait = proxy.split("if (state->waiting_for_search_update)", 1)[1].split(
            "if (state->last_route_plan_at", 1)[0]
        for condition in (
                "map_revision_ > state->search_wait_map_revision",
                "costmap_revision_ > state->search_wait_costmap_revision",
                "observation_count_ > state->search_wait_observation_count",
                "robot_pose_changed", "retry_due"):
            self.assertIn(condition, search_wait)

        blocker_handler = proxy.split("RoutePlan handleFootprintBlocker(", 1)[1].split(
            "RoutePlan prepareManagedDispatch", 1)[0]
        self.assertIn("FootprintBlocker::OUT_OF_BOUNDS", blocker_handler)
        self.assertIn("waitForRouteProgress", blocker_handler)
        self.assertIn("RoutePlanState::ABORT", blocker_handler)

    def test_global_planner_waits_for_temporary_goal_coverage(self):
        plugin = (Path(__file__).resolve().parents[1] / "src" /
                  "global_plan_retry_bt_node.cpp").read_text()
        outcome = plugin.split("BT::NodeStatus handlePlannerOutcome(", 1)[1].split(
            "BT::NodeStatus resolvePlannerFailure", 1)[0]
        self.assertIn('outcome.state == "invalid_goal"', outcome)
        self.assertIn('waitAndRetry(goal, "waiting_for_goal_map_coverage")', outcome)
        self.assertIn("category=invalid_goal_pose", plugin)
        self.assertIn("planner_failed_without_no_path", outcome)

    def test_goal_transform_wait_keeps_goal_after_five_seconds(self):
        proxy = (Path(__file__).resolve().parents[1] / "src" /
                 "navigation_goal_proxy_node.cpp").read_text()
        transform_wait = proxy.split('if (failure_reason == "unknown_goal_frame")', 1)[1]
        transform_wait = transform_wait.split(
            "const auto now = std::chrono::steady_clock::now();", 1)[1].split(
                "if (state && state->goal_normalized)", 1)[0]
        self.assertNotIn("transform_timed_out", transform_wait)
        self.assertNotIn("PublicTerminal::ABORTED", transform_wait)
        self.assertIn("暂停等待定位：保留当前目标", transform_wait)
        self.assertIn("unknown_goal_frame", proxy)
        self.assertIn("getAllFrameNames()", proxy)

    def test_goal_proxy_cancel_and_replacement_stop_the_old_goal(self):
        proxy = (Path(__file__).resolve().parents[1] / "src" /
                 "navigation_goal_proxy_node.cpp").read_text()
        cancel = proxy.split("rclcpp_action::CancelResponse cancelCallback(", 1)[1].split(
            "void acceptedCallback", 1)[0]
        self.assertIn("active_->public_handle != goal", cancel)
        self.assertIn("active_->cancel_requested = true", cancel)
        self.assertIn("async_cancel_goal(internal)", cancel)
        self.assertIn("CancelResponse::ACCEPT", cancel)

        replacement = proxy.split("void acceptedCallback(", 1)[1].split(
            "void readyCallback", 1)[0]
        self.assertIn("active_ = state", replacement)
        self.assertIn("async_cancel_goal(replaced_internal)", replacement)
        self.assertIn("PublicTerminal::ABORTED", replacement)

    def test_pcd_no_return_holes_remain_unknown_without_direct_evidence(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "pcd_to_terrain_map.cpp").read_text()
        self.assertNotIn("inferred_ground_gap", source)
        self.assertNotIn("kMaxInferredGroundGapCells", source)
        self.assertIn("if (stats.count == 0) continue;", source)

    def test_map_navigation_uses_native_action_and_chassis_wiring(self):
        tree_path = (Path(__file__).resolve().parents[1] / "behavior_trees" /
                     "navigate_to_pose_map_guarded_recovery.xml")
        root = ET.parse(tree_path).getroot()
        self.assertEqual(root.find(".//ComputePathToPose").get("goal"), "{planning_goal}")
        self.assertEqual(root.find(".//GlobalPlanRetry").get("candidate_path"),
                         "{candidate_path}")
        self.assertEqual(root.find(".//FollowPath").get("path"), "{path}")
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        self.assertIn('executable="chassis_driver"', launch)
        self.assertNotIn("execution_permit", launch)
        self.assertNotIn("unknown_route_topic", launch)
        self.assertNotIn("navigate_to_pose_identity_proxy_node", launch)
        self.assertNotIn("goal_pose_action_bridge_node", launch)
        nav_launch = (Path(__file__).resolve().parents[1] / "launch" /
                      "nav2_navigation_launch.py").read_text()
        self.assertIn("NavigateToPose", nav_launch)
        self.assertNotIn("navigate_to_pose_identity_proxy_node", nav_launch)

    def test_handoff_node_does_not_redeclare_builtin_use_sim_time(self):
        handoff_source = (Path(__file__).resolve().parents[1] / "src" /
                          "map_odom_handoff_node.cpp").read_text()
        self.assertNotIn('declare_parameter<bool>("use_sim_time"', handoff_source)

    def test_map_to_odom_has_one_authority_and_gicp_publishes_preview_only(self):
        package = Path(__file__).resolve().parents[1]
        handoff = (package / "src" / "map_odom_handoff_node.cpp").read_text()
        gicp = (Path(__file__).resolve().parents[2] / "small_gicp_relocalization" /
                "src" / "small_gicp_relocalization_node.cpp").read_text()
        core = (package / "launch" / "navigation_core.launch.py").read_text()
        script = (Path(__file__).resolve().parents[3] / "tool" /
                  "start_map_navigation.sh").read_text()
        logging = (Path(__file__).resolve().parents[3] / "tool" / "logging.sh").read_text()
        gicp_launch = (package / "launch" /
                       "livox_fastlio_gicp_relocalization.launch.py").read_text()
        self.assertIn("flock(lock_fd_, LOCK_EX | LOCK_NB)", handoff)
        self.assertIn("Another map_odom_handoff publisher is already active", handoff)
        self.assertNotIn("tf2_ros::TransformBroadcaster", gicp)
        self.assertNotIn("sendTransform", gicp)
        self.assertIn("/small_gicp/preview_map_odom", gicp)
        self.assertIn('executable="map_odom_handoff_node"', gicp_launch)
        self.assertEqual(gicp_launch.count('executable="map_odom_handoff_node"'), 1)
        self.assertIn('"enable_handoff", default_value="true"', gicp_launch)
        self.assertIn('"handoff_ready_topic", default_value="/localization/handoff_ready"',
                      gicp_launch)
        self.assertIn('condition=IfCondition(enable_handoff)', gicp_launch)
        self.assertIn('"ready_topic": handoff_ready_topic', gicp_launch)
        self.assertIn('"enable_handoff:=false"', script)
        self.assertEqual(core.count('executable="map_odom_handoff_node"'), 1)
        self.assertIn("OnProcessExit", core)
        self.assertIn("setsid ros2 launch bringup navigation_core.launch.py", script)
        self.assertIn('SESSION_ID="$(date +%s%N)_$$"', script)
        self.assertIn("check_residual_processes || exit 1", script)
        self.assertIn('declare_parameter<double>("update_frequency_hz", 100.0)', handoff)
        self.assertIn(
            'DeclareLaunchArgument("handoff_update_frequency_hz", default_value="100.0")',
            core)
        self.assertIn(
            'HANDOFF_UPDATE_FREQUENCY_HZ="${HANDOFF_UPDATE_FREQUENCY_HZ:-100.0}"',
            script)
        self.assertIn("map_odom_handoff_node", logging)
        self.assertIn("small_gicp_relocalization_node", logging)

    def test_gicp_uses_conservative_thread_and_euclidean_quality_parameters(self):
        gicp_launch = (Path(__file__).resolve().parents[1] / "launch" /
                       "livox_fastlio_gicp_relocalization.launch.py").read_text()
        node_source = (Path(__file__).resolve().parents[2] / "small_gicp_relocalization" /
                       "src" / "small_gicp_relocalization_node.cpp").read_text()
        self.assertIn(
            'DeclareLaunchArgument("tracking_frequency_hz", default_value="1.0")',
            gicp_launch)
        self.assertIn(
            'declare_parameter<double>("tracking_frequency_hz", 1.0)',
            node_source)
        self.assertIn('DeclareLaunchArgument("num_threads", default_value="2"', gicp_launch)
        self.assertIn('"max_iterations", default_value="60"', gicp_launch)
        self.assertIn('DeclareLaunchArgument("stable_frames_required", default_value="4")',
                      gicp_launch)
        self.assertIn('DeclareLaunchArgument("stable_duration", default_value="0.30")',
                      gicp_launch)
        self.assertIn('declare_parameter<int>("stable_frames_required", 4)', node_source)
        self.assertIn('declare_parameter<double>("stable_duration", 0.30)', node_source)
        self.assertIn('DeclareLaunchArgument("max_geometry_rmse_m", default_value="0.18"',
                      gicp_launch)
        self.assertIn('DeclareLaunchArgument("max_geometry_p95_m", default_value="0.38"',
                      gicp_launch)
        self.assertIn('DeclareLaunchArgument("max_multistart_translation_spread", default_value="0.15")',
                      gicp_launch)
        self.assertIn('"max_multistart_translation_spread", 0.15)', node_source)
        self.assertIn('SensorDataQoS().keep_last(1)', node_source)
        self.assertIn('MAP_LOADING', node_source)
        self.assertIn('First registrable scan received', node_source)
        normalized_node_source = " ".join(node_source.split())
        self.assertIn(
            "relocalization_state_ == "
            "small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED && "
            "has_accepted_pose_ ? last_map_odom_ : preview_map_odom_",
            normalized_node_source)
        self.assertIn('const bool preview_candidate_gate', node_source)
        self.assertIn('use_alternate_preview', node_source)
        self.assertIn('max_iterations_ / 2', node_source)
        self.assertIn('alternate_ran', node_source)
        self.assertNotIn('max_mean_gicp_error', node_source)

    def test_map_navigation_wires_terrain_publisher_and_path_display(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        self.assertIn('executable="static_terrain_map_publisher"', launch)
        self.assertIn('executable="nav2_path_display_node"', launch)
        self.assertIn('condition=IfCondition(visualize)', launch)
        self.assertIn('"topic": "/terrain_map/static"', launch)
        self.assertIn('"display_pcd_file": display_pcd_file', launch)
        self.assertNotIn('static_terrain_topic', launch)
        self.assertIn('"map_navigation_terrain.rviz"', launch)

    def test_no_map_nav2_routes_recovery_velocity_through_smoother(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "nav2_theta_teb.launch.py").read_text()
        nav2_launch = (Path(__file__).resolve().parents[1] / "launch" /
                       "nav2_navigation_launch.py").read_text()
        self.assertIn('"nav2_navigation_launch.py"', launch)
        self.assertIn("package='nav2_behaviors'", nav2_launch)
        self.assertIn("remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]", nav2_launch)
        self.assertIn("('cmd_vel_smoothed', 'cmd_vel')", nav2_launch)
        self.assertIn('"navigate_to_pose_action_name": "/navigate_to_pose"', launch)
        self.assertIn("default_value='/navigate_to_pose_internal'", nav2_launch)

    def test_no_map_path_display_uses_direct_nav2_mode(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "nav2_theta_teb.launch.py").read_text()
        node = (Path(__file__).resolve().parents[1] / "src" /
                "nav2_path_display_node.cpp").read_text()
        map_launch = (Path(__file__).resolve().parents[1] / "launch" /
                      "map_navigation.launch.py").read_text()
        self.assertIn('"direct_nav2_mode": True', launch)
        self.assertIn('declare_parameter<bool>("direct_nav2_mode", false)', node)
        self.assertIn('if (!direct_nav2_mode_)', node)
        self.assertIn('"goal_identity_topic": "/navigation/goal_identity"', map_launch)

    def test_no_map_fastdem_terrain_chain_remains_enabled_by_default(self):
        package_root = Path(__file__).resolve().parents[1]
        launch = (package_root / "launch" / "nav2_theta_teb.launch.py").read_text()
        params = yaml.safe_load((package_root / "config" / "nav2_theta_teb.yaml").read_text())
        self.assertIn('"enable_fastdem",\n                default_value="true"', launch)
        self.assertIn('"require_terrain": ParameterValue(enable_fastdem, value_type=bool)',
                      launch)
        self.assertIn('"terrain_status_topic": "/fastdem/local_terrain_valid"', launch)
        gate = (package_root / "src" / "navigation_gate_node.cpp").read_text()
        terrain_layer = (package_root.parent / "nav2" / "nav2_terrain_layer" / "src" /
                         "terrain_layer.cpp").read_text()
        self.assertIn('terrain_status_topic_, rclcpp::QoS(1).reliable().transient_local()', gate)
        self.assertIn('terrain_status_publisher_ = node->create_publisher', terrain_layer)
        for name in ("global_costmap", "local_costmap"):
            costmap = params[name][name]["ros__parameters"]
            self.assertTrue(costmap["terrain_layer"]["enabled"])
        self.assertEqual(
            params["local_costmap"]["local_costmap"]["ros__parameters"]
            ["terrain_layer"]["terrain_status_topic"], "/fastdem/local_terrain_valid")

    def test_no_map_route_gate_matches_theta_star_start_escape(self):
        package_root = Path(__file__).resolve().parents[1]
        planner = yaml.safe_load(
            (package_root / "config" / "nav2_theta_teb.yaml").read_text())[
                "planner_server"]["ros__parameters"]["GridBased"]
        launch = (package_root / "launch" / "nav2_theta_teb.launch.py").read_text()
        proxy = (package_root / "src" / "navigation_goal_proxy_node.cpp").read_text()

        self.assertFalse(planner["allow_unknown"])
        self.assertEqual(planner["start_unknown_escape_radius"], 0.50)
        self.assertIn('"start_unknown_escape_radius": 0.50', launch)
        self.assertIn('"start_unknown_escape_radius", 0.50', proxy)
        self.assertIn("footprintTouchesUnknown(", proxy)

    def test_no_map_route_gate_checks_start_rotation_on_costmap(self):
        proxy = (Path(__file__).resolve().parents[1] / "src" /
                 "navigation_goal_proxy_node.cpp").read_text()
        start = proxy.index("route->points.front() = {robot_x, robot_y};")
        first_waypoint_check = proxy.index(
            "for (std::size_t i = 1; i < route->points.size(); ++i)", start)
        start_guard = proxy[start:first_waypoint_check]

        self.assertIn("if (route->points.size() > 1)", start_guard)
        self.assertIn("std::atan2(", start_guard)
        self.assertIn("assessFootprintRotationSweepWithStartEscape", start_guard)
        self.assertIn(
            "map, costmap, robot_x, robot_y, robot_yaw, first_motion_yaw", start_guard)
        self.assertIn("map_escape_radius, costmap_escape_radius", start_guard)
        self.assertIn("waitForRouteProgress(", start_guard)
        self.assertIn(
            'map_start_rotation.clear ? "global_costmap" : "map"', start_guard)

    def test_frontier_exploration_is_opt_in_and_updates_the_fused_map(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        fusion = (Path(__file__).resolve().parents[1] / "src" /
                  "global_map_fusion_node.cpp").read_text()
        self.assertIn('"enable_frontier_exploration", default_value="false"', launch)
        self.assertIn('executable="frontier_explorer_node"', launch)
        self.assertIn('condition=IfCondition(enable_frontier_exploration)', launch)
        self.assertNotIn('"nonground_topic": "/patchworkpp/nonground"', launch)
        self.assertIn('"global_clearing_topic": "/obstacles/global_clearing"', launch)
        self.assertNotIn('"/obstacles/global_blocking"', fusion)
        self.assertIn('"ground_veto_topic": "/obstacles/ground_veto"', launch)
        self.assertIn('"clearing_ray_topic": "/cloud_registered_body"', launch)
        self.assertIn('"online_obstacle_clear_observations": 5', launch)
        self.assertIn('"raytrace_min_range": 0.1', launch)
        self.assertIn('"raytrace_max_range": 20.0', launch)
        self.assertIn("max_map_extension_m", launch)
        self.assertIn("max_map_cells", launch)
        self.assertIn("selectFrontierGoal", (Path(__file__).resolve().parents[1] /
                                               "include" / "bringup" /
                                               "frontier_selection.hpp").read_text())
        explorer = (Path(__file__).resolve().parents[1] / "src" /
                    "frontier_explorer_node.cpp").read_text()
        self.assertIn('"map_updates_topic": "/map_updates"', launch)
        self.assertIn("mapUpdateReachedCostmap()", explorer)
        self.assertIn("global_costmap_received_ > map_received_", explorer)
        self.assertIn("operator_paused_", explorer)
        self.assertIn("/navigation/frontier_exploration/stop", explorer)
        self.assertIn("/navigation/frontier_exploration/resume", explorer)
        self.assertIn("ground_synchronizer_", fusion)
        self.assertIn("ground_veto_sub_", fusion)
        self.assertNotIn("obstacleCallback", fusion)
        self.assertIn("globalClearingCallback", fusion)
        self.assertIn("excludeCurrentObstacleHitsFromRayClears", fusion)
        self.assertIn("obstacle_hit_cells", fusion)
        self.assertIn("latest_obstacle_hit_stamp_ns_", fusion)
        self.assertIn("obstacleHitVetoesClearEvent", fusion)
        self.assertIn("clearingRayCallback", fusion)
        self.assertIn("retryPendingGlobalClearEvents", fusion)
        self.assertIn("retryAvailable", fusion)
        self.assertIn("FreshRetryQueue", fusion)
        self.assertIn("growMapForGround", fusion)
        self.assertIn("clear_observations_", fusion)

    def test_rviz_panel_rechecks_all_goal_handles_after_spinning_callbacks(self):
        source = (Path(__file__).resolve().parents[3] / "src" / "nav2_rviz_plugins" /
                  "src" / "nav2_panel.cpp").read_text()
        timer_event = source.split("Nav2Panel::timerEvent(QTimerEvent * event)", 1)[1]
        timer_event = timer_event.split("\n}\n", 1)[0]
        handles = (
            "waypoint_follower_goal_handle_",
            "nav_through_poses_goal_handle_",
            "navigation_goal_handle_",
        )
        self.assertEqual(timer_event.count("const auto goal_handle = "), 3)
        for handle in handles:
            self.assertRegex(
                timer_event,
                re.compile(
                    r"rclcpp::spin_some\(client_node_\);\s+"
                    r"const auto goal_handle = " + re.escape(handle) + r";\s+"
                    r"if \(!goal_handle\).*?get_status\(\)",
                    re.DOTALL,
                ),
            )
        self.assertEqual(timer_event.count("timer_.stop();"), 9)
        self.assertEqual(timer_event.count("QActionState::INACTIVE"), 9)
        cancellation = source.split("Nav2Panel::onCancelButtonPressed()", 1)[1]
        cancellation = cancellation.split("\n}\n", 1)[0]
        self.assertEqual(cancellation.count("get_goal_id() =="), 3)
        self.assertIn("if (!navigation_goal_handle_ && !waypoint_follower_goal_handle_ &&", cancellation)

    def test_navigation_panel_keeps_goal_matched_rejection_and_ends_planner_wait(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "navigation_status_panel.cpp").read_text()
        self.assertIn('"static_goal_unknown"', source)
        self.assertIn('"category=global_no_path "', source)
        self.assertIn("diagnosticMatchesActiveGoal", source)
        self.assertIn("goal_rejection_cache_", source)
        self.assertIn("have_current_goal_rejection_", source)
        self.assertIn("finishPlannerWaitIfNavigationEnded", source)
        self.assertIn("messages_->clear();", source)

    def test_navigation_ready_heartbeat_survives_search_periods_and_false_cancels(self):
        bringup = Path(__file__).resolve().parents[1]
        heartbeat = (bringup / "include" / "bringup" / "status_heartbeat.hpp").read_text()
        gate = (bringup / "src" / "navigation_gate_node.cpp").read_text()
        explorer = (bringup / "src" / "frontier_explorer_node.cpp").read_text()
        gate_tick = gate.split("timer_ = create_wall_timer", 1)[1].split("});", 1)[0]
        ready_callback = explorer.split("navigation_ready_sub_ =", 1)[1].split(
            "health_sub_ =", 1)[0]
        heartbeat_test = (Path(__file__).resolve().parents[1] / "test" /
                          "test_navigation_gate_state.cpp").read_text()
        self.assertIn("kNavigationReadyHeartbeatPeriod{250}", heartbeat)
        self.assertIn("kNavigationReadyHeartbeatTimeoutSec = 1.0", heartbeat)
        self.assertIn("publishNavigationReadyHeartbeatIfDue();", gate_tick)
        self.assertIn("publishReady(navigation_ready_)", gate)
        self.assertIn("navigationReadyHeartbeatIsFresh", explorer)
        self.assertIn("if (!navigation_ready_ && goal_in_flight_)", ready_callback)
        self.assertIn("requestGoalCancel(\"导航门控已暂停\")", ready_callback)
        self.assertIn("2.0 * search_period_sec", heartbeat_test)
        self.assertIn("navigationReadyHeartbeatIsFresh(false, 0.0)", heartbeat_test)

    def test_legacy_navigation_handoff_matches_volatile_fastlio_health(self):
        bringup = Path(__file__).resolve().parents[1]
        gate = (bringup / "src" / "navigation_gate_node.cpp").read_text()
        launch = (bringup / "launch" / "nav2_theta_teb.launch.py").read_text()
        proxy = (bringup / "src" / "navigation_goal_proxy_node.cpp").read_text()
        legacy_handoff = gate.split(
            "    } else {\n      handoff_subscription_ = create_subscription", 1)[1].split(
                "    }\n\n    if (require_fastlio_health_)", 1)[0]
        ready_callback = proxy.split("void readyCallback(const bool ready)", 1)[1].split(
            "void dispatchWhenReady()", 1)[0]
        self.assertIn('"use_relocalization_status": False', launch)
        self.assertIn('"localization_ready_topic": "/fastlio/health"', launch)
        self.assertIn("localization_ready_topic, rclcpp::QoS(1)", legacy_handoff)
        self.assertNotIn("transient_local", legacy_handoff)
        self.assertIn('"internal_action": "/navigate_to_pose_internal"', launch)
        self.assertIn("if (ready) {\n      dispatchWhenReady();", ready_callback)
        self.assertIn("internal_client_->async_send_goal(dispatched_goal, options);", proxy)

    def test_frontier_terrain_gate_reuses_full_source_and_confidence_validation(self):
        bringup = Path(__file__).resolve().parents[1]
        explorer = (bringup / "src" / "frontier_explorer_node.cpp").read_text()
        launch = (bringup / "launch" / "map_navigation.launch.py").read_text()
        chassis_tests = (bringup / "test" / "test_chassis_driver.cpp").read_text()
        callback = explorer.split("terrain_sub_ =", 1)[1].split(
            "action_client_ =", 1)[0]
        gate = explorer.split("std::string dataGateFailure() const", 1)[1].split(
            "void checkDataGate()", 1)[0]
        self.assertIn("terrain_message_is_current", callback)
        self.assertIn('"elevation"', callback)
        self.assertIn("std::isfinite(value)", callback)
        self.assertIn("get_clock()->now().nanoseconds()", callback)
        self.assertIn("stamp_ns == last_terrain_stamp_ns_", callback)
        self.assertIn("terrain_message_is_current", gate)
        self.assertIn("terrain_receive_age <= terrain_config_.max_age", gate)
        self.assertIn('"terrain_max_future_stamp_offset": 0.1', launch)
        self.assertIn('"terrain_min_confident_cells": 20', launch)
        self.assertIn("TerrainGateRejectsSparseAndLethalOutlierFrames", chassis_tests)
        self.assertIn("TerrainGateRequiresCurrentTrustedGeometry", chassis_tests)

    def test_goal_rejection_cache_keeps_logs_until_status_and_identity_match(self):
        bringup = Path(__file__).resolve().parents[1]
        panel = (bringup / "src" / "navigation_status_panel.cpp").read_text()
        cache_test = (bringup / "test" / "test_goal_rejection_cache.cpp").read_text()
        new_goal_callback = panel.split("navigation_goal_sub_ =", 1)[1].split(
            "goal_identity_sub_ =", 1)[0]
        public_identity_callback = panel.split("goal_identity_sub_ =", 1)[1].split(
            "goal_pose_identity_sub_ =", 1)[0]
        pose_identity_callback = panel.split("goal_pose_identity_sub_ =", 1)[1].split(
            "local_wait_sub_ =", 1)[0]
        active_identity = panel.split("void updateActiveGoalPoseIdentity()", 1)[1].split(
            "static bool isGoalRejectionDiagnostic", 1)[0]
        show_rejection = panel.split("bool showGoalRejectionDiagnostic(", 1)[1].split(
            "static QString goalRejectionSummary", 1)[0]
        self.assertNotIn("goal_rejection_cache_.clear()", new_goal_callback)
        self.assertIn("updateActiveGoalPoseIdentity();", new_goal_callback)
        self.assertIn("updateActiveGoalPoseIdentity();", public_identity_callback)
        self.assertIn("updateActiveGoalPoseIdentity();", pose_identity_callback)
        self.assertIn("takeForGoal", active_identity)
        self.assertIn("goal_rejection_cache_.remember", show_rejection)
        self.assertIn("std::next_permutation", cache_test)
        self.assertIn("KeepsOtherGoalsPendingAndBoundsStorage", cache_test)

    def test_late_goal_status_does_not_overwrite_replayed_rejection_text(self):
        bringup = Path(__file__).resolve().parents[1]
        panel = (bringup / "src" / "navigation_status_panel.cpp").read_text()
        cache_test = (bringup / "test" / "test_goal_rejection_cache.cpp").read_text()
        new_goal_callback = panel.split("navigation_goal_sub_ =", 1)[1].split(
            "goal_identity_sub_ =", 1)[0]
        begin_wait = new_goal_callback.index("planner_display_state_.beginGoal")
        replay_pending = new_goal_callback.index("updateActiveGoalPoseIdentity();")
        self.assertLess(begin_wait, replay_pending)
        self.assertIn("planner_display_state_.showRejection", panel)
        self.assertIn("planner_display_state_.finishIfWaiting", panel)
        self.assertIn("LateGoalStatusAndAbortPreserveCachedRejectionText", cache_test)
        self.assertIn("EXPECT_EQ(display.text(), rejection_text)", cache_test)

    def test_static_terrain_display_tracks_expanding_map_and_live_elevation(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "static_terrain_map_publisher.cpp").read_text()
        terrain_layer = (Path(__file__).resolve().parents[2] / "nav2" /
                         "nav2_terrain_layer" / "src" /
                         "static_terrain_layer.cpp").read_text()
        self.assertIn('"/map"', source)
        self.assertIn('"/fastdem/postprocess/gridmap"', source)
        self.assertIn("extendDisplayMapToOccupancyGrid", source)
        self.assertIn("mergeOnlineDisplayTerrain", source)
        self.assertIn("std::round(offset_x)", terrain_layer)
        self.assertIn("std::round(offset_y)", terrain_layer)

    def test_static_map_startup_source_and_stale_delta_ordering(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "static_terrain_map_publisher.cpp").read_text()
        full_map = source.split("void processFullMapPair(", 1)[1].split(
            "void discardCoveredPendingMapUpdates(", 1)[0]
        epoch_reset = full_map.split("if (disposition == bringup::StaticPcdFullMapDisposition::ACCEPT_NEW_EPOCH)",
                                     1)[1].split("map_publisher_epoch_ =", 1)[0]
        self.assertIn("discardStaleStartupSource(snapshot)", full_map)
        self.assertIn("latest_full_navigation_map_ = snapshot", full_map)
        watermark_only = full_map.split(
            "if (disposition == bringup::StaticPcdFullMapDisposition::ADVANCE_WATERMARK_ONLY)",
            1)[1].split("return;", 1)[0]
        self.assertIn("latest_full_navigation_map_ = snapshot", watermark_only)
        self.assertNotIn("latest_occupancy_source_.reset()", epoch_reset)

        source_filter = source.split("void discardStaleStartupSource(", 1)[1].split(
            "void processPendingMapUpdates(", 1)[0]
        self.assertIn("shouldDiscardStaticPcdOccupancySource", source_filter)

        startup_evidence = source.split("void applyStartupPriorClearanceEvidence(", 1)[1].split(
            "bool recordStaticPcdClear(", 1)[0]
        self.assertIn("collectStartupPriorClearanceCandidates", startup_evidence)
        self.assertIn("*latest_full_navigation_map_", startup_evidence)
        self.assertIn("*latest_navigation_map_", startup_evidence)

        update_callback = source.split("void mapUpdateCallback(", 1)[1].split(
            "void drainMapGeometryQueues(", 1)[0]
        self.assertLess(
            update_callback.index("staticPcdMapUpdateCoveredByState"),
            update_callback.index("PendingMapUpdate pending"))

        pending_updates = source.split("void processPendingMapUpdates(", 1)[1].split(
            "void invalidateMapDeltaChain(", 1)[0]
        self.assertLess(
            pending_updates.index("staticPcdMapUpdateCoveredByState"),
            pending_updates.index("if (!pending.geometry) return"))

    def test_map_navigation_rviz_has_2_5d_terrain_and_nav_goal_tool(self):
        config = yaml.safe_load((RVIZ / "map_navigation.rviz").read_text())
        manager = config["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        displays = manager["Displays"]
        topics = {
            display.get("Name"): display.get("Topic", {}).get("Value")
            for display in displays
        }
        self.assertEqual(topics["Smooth Elevation Surface"], "/terrain_map/static")
        self.assertEqual(topics["Online Local Terrain"], "/fastdem/postprocess/gridmap")
        self.assertEqual(
            topics["全局膨胀代价"], "/navigation_display/global_inflation")
        self.assertEqual(
            topics["局部膨胀代价"], "/navigation_display/local_inflation")
        self.assertNotIn("局部总代价", topics)
        by_name = {display["Name"]: display for display in displays}
        surface = by_name["Smooth Elevation Surface"]
        self.assertFalse(surface["Enabled"])
        self.assertFalse(surface["Show Grid Lines"])
        self.assertEqual(surface["Color Layer"], "display_relief")
        self.assertEqual(surface["Min Intensity"], 0)
        self.assertEqual(surface["Max Intensity"], 1)
        self.assertEqual(surface["Height Layer"], "display_elevation")
        self.assertEqual(surface["Height Transformer"], "GridMapLayer")
        self.assertFalse(surface["Use Rainbow"])
        self.assertEqual(surface["Min Color"], "200; 200; 200")
        self.assertEqual(surface["Max Color"], "200; 200; 200")
        pcd = by_name["Height-colored PCD Map"]
        self.assertTrue(pcd["Enabled"])
        self.assertEqual(pcd["Topic"]["Value"], "/terrain_map/static_cloud")
        terrain_config = yaml.safe_load(
            (RVIZ / "map_navigation_terrain.rviz").read_text())["Visualization Manager"]
        terrain_displays = {
            display.get("Name"): display
            for display in terrain_config["Displays"]
        }
        self.assertTrue(terrain_displays["Smooth Elevation Surface"]["Enabled"])
        self.assertFalse(terrain_displays["Height-colored PCD Map"]["Enabled"])
        self.assertFalse(by_name["Online Local Terrain"]["Enabled"])
        for name in ("全局膨胀代价", "局部膨胀代价"):
            display = by_name[name]
            self.assertEqual(display["Class"], "rviz_default_plugins/MarkerArray")
            self.assertEqual(display["Topic"]["Durability Policy"], "Transient Local")
        self.assertFalse(by_name["全局膨胀代价"]["Enabled"])
        self.assertFalse(by_name["局部膨胀代价"]["Enabled"])
        nav_config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        global_costmap = nav_config["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertLessEqual(global_costmap["inflation_layer"]["inflation_radius"], 0.25)
        self.assertNotIn("/terrain_map/static_cloud", topics.values())
        self.assertNotIn("/obstacles/blocking", topics.values())
        self.assertNotIn("/global_costmap/costmap", topics.values())
        self.assertNotIn("/local_costmap/costmap", topics.values())
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "static_terrain_map_publisher.cpp").read_text()
        self.assertIn('"/global_costmap/costmap"', source)
        self.assertIn('"/local_costmap/costmap"', source)
        self.assertIn('staticDisplayHeightAboveGround', source)
        self.assertIn('Marker::TRIANGLE_LIST', source)
        self.assertIn(
            'local_inflation_radius_ = declare_parameter<double>("local_inflation_radius", 0.25)',
            source)
        self.assertIn(
            'costmap, local ? local_inflation_radius_ : global_inflation_radius_)', source)
        self.assertIn('bringup::inflationCells(', source)
        self.assertIn('inflationColor(cell.cost)', source)
        self.assertIn("marker.colors.push_back(color)", source)
        self.assertIn("cell.x + (i == 1 || i == 2 ? 1.0 : 0.0)", source)
        self.assertIn('"/local_costmap/costmap_updates"', source)
        self.assertIn('"/global_costmap/costmap_updates"', source)
        self.assertNotIn('marker.lifetime =', source)
        self.assertEqual(manager["Views"]["Current"]["Class"], "rviz_default_plugins/Orbit")
        tools = [tool.get("Class") for tool in manager["Tools"]]
        self.assertIn("nav2_rviz_plugins/GoalTool", tools)
        self.assertNotIn("rviz_default_plugins/SetGoal", tools)
        panels = [panel.get("Class") for panel in yaml.safe_load(
            (RVIZ / "map_navigation.rviz").read_text())["Panels"]]
        self.assertIn("nav2_rviz_plugins/Navigation 2", panels)
        self.assertIn("bringup/NavigationStatus", panels)
        plugin = ET.parse(RVIZ.parent / "navigation_panel_plugin.xml").getroot()
        self.assertEqual(plugin.find("class").get("name"), "bringup/NavigationStatus")
        self.assertEqual(plugin.find("class").get("base_class_type"), "rviz_common::Panel")

    def test_map_navigation_restores_staged_rviz_startup_order(self):
        script = (Path(__file__).resolve().parents[3] / "tool" /
                  "start_map_navigation.sh").read_text()
        self.assertIn('VISUALIZE="${VISUALIZE:-true}"', script)
        self.assertIn('GICP_VISUALIZE="${GICP_VISUALIZE:-true}"', script)
        self.assertIn('GICP_RVIZ_CONFIG="${GICP_RVIZ_CONFIG:-}"', script)
        self.assertIn('capture_cores:=*) CAPTURE_CORES=', script)
        self.assertIn("ulimit -c unlimited", script)
        startup_marker = 'echo "[*] Starting map navigation gate before initial localization"'
        main_flow = script[script.index(startup_marker):]

        def navigation_starts_before_localization(source):
            return source.index("start_map_navigation_stage || exit $?") < \
                source.index("run_localization_cycle initial")

        self.assertTrue(navigation_starts_before_localization(main_flow))
        navigation_call = "start_map_navigation_stage || exit $?"
        localization_call = "run_localization_cycle initial"
        swap_marker = "__STARTUP_ORDER_SWAP__"
        reversed_main_flow = main_flow.replace(navigation_call, swap_marker, 1)
        reversed_main_flow = reversed_main_flow.replace(localization_call, navigation_call, 1)
        reversed_main_flow = reversed_main_flow.replace(swap_marker, localization_call, 1)
        self.assertFalse(navigation_starts_before_localization(reversed_main_flow))

        for filename in ("map_navigation.rviz", "map_navigation_terrain.rviz"):
            config = yaml.safe_load((RVIZ / filename).read_text())
            displays = {display.get("Name"): display
                        for display in config["Visualization Manager"]["Displays"]}
            self.assertEqual(displays["base_link axes"]["Reference Frame"], "base_link")
            self.assertEqual(displays["base_link axes"]["Length"], 1)
            self.assertEqual(displays["base_link axes"]["Radius"], 0.03)
            self.assertTrue(displays["TF"]["Show Arrows"])
            self.assertTrue(displays["TF"]["Show Axes"])
            self.assertTrue(displays["TF"]["Show Names"])
            self.assertEqual(displays["Localization Preview"]["Topic"]["Value"],
                             "/small_gicp/pose")
            self.assertIn("bringup/NavigationStatus",
                          [panel.get("Class") for panel in config["Panels"]])
        status_panel = (Path(__file__).resolve().parents[1] / "src" /
                        "navigation_status_panel.cpp").read_text()
        self.assertIn('health_ = new QLabel("定位：等待健康状态", this);', status_panel)
        self.assertNotIn("GICP 定位", status_panel)
        self.assertNotIn("relocalization_sub_", status_panel)
        relocalization_node = (Path(__file__).resolve().parents[3] / "src" /
                               "small_gicp_relocalization" / "src" /
                               "small_gicp_relocalization_node.cpp").read_text()
        self.assertIn("status.degraded = status.valid &&", relocalization_node)
        self.assertIn("has_pending_tracking_candidate_", relocalization_node)

    def test_navigation_shutdown_signals_launch_before_process_group(self):
        script = (Path(__file__).resolve().parents[3] / "tool" /
                  "start_map_navigation.sh").read_text()
        shutdown = script.split("shutdown_registered_processes() {", 1)[1].split(
            "\n}\n\ncleanup()", 1)[0]
        chassis_launch_signal = shutdown.index('kill -INT "${nav_pid}"')
        other_launch_signals = shutdown.index('for group_id in "${launch_group_ids[@]}"')
        self.assertLess(chassis_launch_signal, other_launch_signals)
        self.assertIn("SECONDS + 2", shutdown)
        self.assertIn("SECONDS + 4", shutdown)
        self.assertNotIn("SECONDS + 15", script)
        self.assertIn("trap 'handle_shutdown_signal SIGINT' INT", script)
        self.assertIn("force_kill_registered_processes", script)
        self.assertNotIn("check_residual_processes --terminate", script)
        map_launch = (Path(__file__).resolve().parents[1] / "launch" /
                      "map_navigation.launch.py").read_text()
        self.assertIn("respawn=True", map_launch)

    def test_map_navigation_entrypoint_closes_localization_rviz_after_handoff(self):
        script = (Path(__file__).resolve().parents[3] / "tool" /
                  "start_map_navigation.sh").read_text()
        self.assertIn('GICP_VISUALIZE="${GICP_VISUALIZE:-true}"', script)
        self.assertIn('GICP_RVIZ_CONFIG="${GICP_RVIZ_CONFIG:-}"', script)
        self.assertIn(
            'GICP_RVIZ_CONFIG="${rviz_prefix}/share/bringup/rviz/gicp_relocalization.rviz"',
            script)
        self.assertIn('"visualize:=${GICP_VISUALIZE}"', script)
        self.assertIn('nav_args+=("visualize:=${VISUALIZE}" "rviz_config:=${RVIZ_CONFIG}")', script)
        localization_cycle = script.split("run_localization_cycle() {", 1)[1].split(
            "\n}\n\nconfigure_core_dumps", 1)[0]
        self.assertIn(
            "start_gicp_rviz\n    start_registered_launch ros2 launch bringup "
            "livox_fastlio_gicp_relocalization.launch.py",
            localization_cycle)
        self.assertIn(
            'echo "[*] Fresh localization handoff received; closing GICP RViz"\n'
            "        stop_handoff_ready_watcher\n"
            "        complete_first_localization_handoff",
            localization_cycle)
        self.assertIn(
            "complete_first_localization_handoff() {\n"
            "  stop_gicp_rviz\n"
            "  navigation_handoff_completed=true",
            script)

    def test_map_navigation_entrypoint_defines_cached_conversion(self):
        script = (Path(__file__).resolve().parents[3] / "tool" /
                  "start_map_navigation.sh").read_text()
        self.assertIn("pcd_to_terrain_map", script)
        self.assertIn("terrain_map.npz", script)
        self.assertIn("display_map.pcd", script)
        self.assertIn("display_voxel_size:=", script)
        self.assertIn("map.yaml", script)
        self.assertIn("rebuild_map:=", script)
        self.assertIn("min_ground_neighbors:=", script)
        self.assertIn("min_ground_component_cells:=", script)
        self.assertIn("ground_component_height_band:=", script)
        self.assertIn('ros2 pkg prefix bringup 2>/dev/null || true', script)
        self.assertIn("sha256sum", script)
        self.assertIn("flock -w 0", script)
        self.assertIn('DISPLAY_PCD="${DISPLAY_PCD:-true}"', script)
        self.assertIn('if [[ "${DISPLAY_PCD}" == "true" ]]; then', script)
        self.assertIn('map_navigation_terrain.rviz', script)
        self.assertIn('nav_args+=("display_pcd_file:=${display_pcd_file}")', script)
        self.assertIn('"pcd_file:=${PCD_FILE}"', script)
        self.assertIn('"cloud_topic:=/cloud_registered_body"', script)
        self.assertIn('"skip_voxel_downsampling:=true"', script)
        self.assertNotIn("relocalization_map.pcd", script)
        repository = Path(__file__).resolve().parents[3]
        bbs = yaml.safe_load(
            (repository / "src" / "bbs3d_ros2" / "config" /
             "bbs3d_relocalization.yaml").read_text())
        self.assertEqual(bbs["lidar_topic_name"], "/cloud_registered_body")
        self.assertEqual(bbs["tar_leaf_size"], 0.0)
        self.assertEqual(bbs["src_leaf_size"], 0.0)
        gicp_launch = (repository / "src" / "bringup" / "launch" /
                       "livox_fastlio_gicp_relocalization.launch.py").read_text()
        self.assertIn('"skip_voxel_downsampling": skip_voxel_downsampling', gicp_launch)
        gicp_node = (repository / "src" / "small_gicp_relocalization" / "src" /
                     "small_gicp_relocalization_node.cpp").read_text()
        self.assertIn("copyWithoutVoxelDownsampling(raw_map)", gicp_node)
        self.assertIn("copyWithoutVoxelDownsampling(raw_source)", gicp_node)
        self.assertIn("ros2 topic echo --no-daemon --once", script.replace("\\\n", ""))
        self.assertIn("--qos-durability transient_local", script)
        self.assertIn('"${LOCALIZATION_READY_TOPIC}" std_msgs/msg/Bool', script)
        self.assertIn("handoff_status=$?", script)
        self.assertIn('exit "${handoff_status}"', script)
        self.assertNotIn('timeout 10 ros2 topic echo --once "${LOCALIZATION_READY_TOPIC}"', script)

    def test_map_navigation_has_explicit_safe_initial_pose_option(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        self.assertIn('"publish_initial_pose", default_value="false"', launch)
        self.assertIn('executable="publish_initial_pose.py"', launch)
        self.assertIn('"initial_pose_x", default_value="0.0"', launch)

    def test_navigation_gate_waits_for_localized_status(self):
        gate = (Path(__file__).resolve().parents[1] / "src" /
                "navigation_gate_node.cpp").read_text()
        self.assertIn("message->valid", gate)
        self.assertIn("RelocalizationStatus::LOCALIZED", gate)
        self.assertIn("Waiting for relocalization success", gate)
        self.assertIn("Relocalization gate opening Nav2 lifecycle", gate)

    def test_navigation_pause_reason_is_published_and_shown_in_status_panel(self):
        gate = (Path(__file__).resolve().parents[1] / "src" /
                "navigation_gate_node.cpp").read_text()
        panel = (Path(__file__).resolve().parents[1] / "src" /
                 "navigation_status_panel.cpp").read_text()
        self.assertIn('"Navigation gate PAUSED: %s"', gate)
        self.assertIn('text.startsWith("Navigation gate PAUSED:")', panel)
        self.assertIn('last_navigation_pause_reason_', panel)
        self.assertIn("navigationPauseHint(last_navigation_pause_reason_)", panel)
        self.assertIn("检查 FAST-LIO 健康状态以及 LiDAR/IMU 输入。", panel)
        self.assertIn("检查 FastDEM 地形状态和数据时效。", panel)

    def test_local_wait_status_is_goal_scoped_and_terminal_status_wins(self):
        package = Path(__file__).resolve().parents[1]
        panel = (package / "src" / "navigation_status_panel.cpp").read_text()
        retry = (package / "src" / "follow_path_retry_bt_node.cpp").read_text()
        teb = (Path(__file__).resolve().parents[2] / "nav2" / "teb_local_planner" /
               "src" / "teb_local_planner_ros.cpp").read_text()
        status_logic = panel.split("void refreshExecutionStatus()", 1)[1].split(
            "void showPlannerStatus", 1)[0]
        self.assertLess(
            status_logic.index("STATUS_SUCCEEDED"),
            status_logic.index("state=blocked_waiting_retry"))
        self.assertIn("STATUS_CANCELED", status_logic)
        self.assertIn("localStatusMatchesCurrentGoal()", status_logic)
        self.assertIn('"/navigation/goal_identity"', panel)
        self.assertIn('";goal_id=" << goalIdText(goal_)', retry)
        self.assertIn('publishLocalStatus("executing", "follow_path_started")', retry)
        self.assertIn('publishLocalStatus("stopped", "behavior_tree_halted")', retry)
        self.assertIn("dwb_critics::LineIterator line", teb)
        self.assertIn("cost == nav2_costmap_2d::LETHAL_OBSTACLE", teb)
        self.assertIn("cost == nav2_costmap_2d::NO_INFORMATION", teb)
        self.assertNotIn("cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE", teb)

    def test_map_navigation_starts_rviz_by_default(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        self.assertIn('"visualize", default_value="true"', launch)
        self.assertNotIn('"visualize": "false"', launch)
        self.assertNotIn('"gicp_visualize"', launch)

    def test_chassis_can_preflight_requires_active_controller_and_bitrate(self):
        launch_path = Path(__file__).resolve().parents[1] / "launch" / "map_navigation.launch.py"
        spec = importlib.util.spec_from_file_location("map_navigation_can_launch", launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        ready_output = (
            "3: can3: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP\n"
            "    link/can\n"
            "    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0\n"
            "      bitrate 500000 sample-point 0.750\n"
        )
        self.assertEqual(module.inspect_chassis_can_interface(ready_output, 0),
                         (True, "ERROR-ACTIVE"))
        for state in ("BUS-OFF", "STOPPED", "ERROR-PASSIVE", "ERROR-WARNING"):
            with self.subTest(state=state):
                output = ready_output.replace("ERROR-ACTIVE", state)
                self.assertEqual(module.inspect_chassis_can_interface(output, 0),
                                 (False, state))

        self.assertEqual(module.inspect_chassis_can_interface(
            ready_output.replace("can state ERROR-ACTIVE", "can state"), 0),
            (False, "UNKNOWN"))
        self.assertFalse(module.inspect_chassis_can_interface(
            ready_output.replace("UP,LOWER_UP", "LOWER_UP"), 0)[0])
        self.assertFalse(module.inspect_chassis_can_interface(
            ready_output.replace("bitrate 500000 sample-point 0.750", ""), 0)[0])
        self.assertFalse(module.inspect_chassis_can_interface(ready_output, 1)[0])

    def test_map_navigation_enables_live_local_terrain_and_gates_on_fresh_data(self):
        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        self.assertEqual(
            config["local_costmap"]["local_costmap"]["ros__parameters"]
            ["terrain_layer"]["terrain_status_topic"], "/fastdem/terrain_valid")
        launch_path = Path(__file__).resolve().parents[1] / "launch" / "map_navigation.launch.py"
        launch_source = launch_path.read_text()
        launch_ast = ast.parse(launch_source)
        rewritten_yaml = next(
            node for node in ast.walk(launch_ast)
            if isinstance(node, ast.Call) and
            isinstance(node.func, ast.Name) and node.func.id == "RewrittenYaml")
        rewrites = next(keyword.value for keyword in rewritten_yaml.keywords
                        if keyword.arg == "param_rewrites")
        rewrite = dict(zip(
            (ast.literal_eval(key) for key in rewrites.keys), rewrites.values))
        enabled_key = "local_costmap.local_costmap.ros__parameters.terrain_layer.enabled"
        self.assertIsInstance(rewrite[enabled_key], ast.Name)
        self.assertEqual(rewrite[enabled_key].id, "enable_local_terrain")
        local_terrain_argument = next(
            node for node in ast.walk(launch_ast)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
            node.func.id == "DeclareLaunchArgument" and node.args and
            ast.literal_eval(node.args[0]) == "enable_local_terrain")
        local_terrain_default = next(
            keyword.value for keyword in local_terrain_argument.keywords
            if keyword.arg == "default_value")
        self.assertEqual(ast.literal_eval(local_terrain_default), "true")
        self.assertTrue(any(
            keyword.arg == "convert_types" and isinstance(keyword.value, ast.Constant) and
            keyword.value.value is True for keyword in rewritten_yaml.keywords))

        spec = importlib.util.spec_from_file_location("map_navigation_launch", launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        self.assertTrue(description.entities)
        self.assertIn('"require_terrain_status": ParameterValue(require_terrain, value_type=bool)',
                      launch_source)
        self.assertIn('"require_terrain", default_value="false"', launch_source)
        self.assertIn('"terrain_status_topic": "/fastdem/terrain_valid"', launch_source)
        self.assertIn('"terrain_valid_topic": "/navigation/terrain_valid"', launch_source)

    def test_online_terrain_status_topics_do_not_collide(self):
        config = yaml.safe_load((CONFIG / "nav2_theta_teb.yaml").read_text())
        global_layer = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_layer = config["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertEqual(
            global_layer["terrain_layer"]["terrain_status_topic"],
            "/fastdem/global_terrain_valid")
        self.assertEqual(
            local_layer["terrain_layer"]["terrain_status_topic"],
            "/fastdem/local_terrain_valid")
        self.assertNotEqual(
            global_layer["terrain_layer"]["terrain_status_topic"],
            local_layer["terrain_layer"]["terrain_status_topic"])

    def test_ground_fusion_only_pairs_same_source_scan(self):
        fusion = (Path(__file__).resolve().parents[1] / "src" /
                  "global_map_fusion_node.cpp").read_text()
        self.assertIn("sync_policies::ExactTime<Cloud, Cloud>", fusion)
        self.assertIn("ObstacleRaySyncPolicy", fusion)
        self.assertIn("obstacle_ray_synchronizer_", fusion)
        self.assertIn("obstacle_sub_.subscribe(this, ground_veto_topic_", fusion)
        self.assertIn("GroundVetoSyncPolicy", fusion)
        self.assertIn("obstacle_ray_synchronizer_", fusion)
        self.assertIn("clearing_ray_sub_.subscribe", fusion)
        self.assertIn("mapPoints(message, map_from_obstacles, false)", fusion)
        self.assertIn("obstacle_evidence_sub_ = create_subscription<Cloud>", fusion)
        self.assertIn("ground_veto_topic_, rclcpp::QoS(10).reliable()", fusion)
        self.assertIn("create_client<std_srvs::srv::Trigger>", fusion)
        self.assertIn("requestObstacleEvidenceFenceLocked", fusion)
        self.assertIn("confirmed_obstacle_evidence_fence_", fusion)
        self.assertIn("ObstacleEvidenceSequenceTracker", fusion)
        self.assertIn("sequenceRangeProcessed", fusion)
        self.assertIn("ClearEventFenceRefreshTracker", fusion)
        self.assertIn("sequenceRangeReady", fusion)
        self.assertIn("eventsNeedingFreshFenceThrough", fusion)
        self.assertIn("request_clear_event_id", fusion)
        self.assertNotIn("clear_event_generation_", fusion)
        evidence_tests = (Path(__file__).resolve().parents[1] / "test" /
                          "test_map_fusion_evidence.cpp").read_text()
        self.assertIn("RefreshesFenceAfterWaitingSequenceRangeRecovers", evidence_tests)
        self.assertIn("second_published_but_unprocessed", evidence_tests)
        self.assertIn("obstacleHitVetoesRayClear", fusion)
        self.assertIn("obstacleEvidenceCallback", fusion)
        self.assertIn("recordObstacleHitEvidence", fusion)
        self.assertIn("pending_obstacle_hit_frames_", fusion)
        self.assertIn("processed_obstacle_frame_stamps_", fusion)
        self.assertIn("obstacle_evidence_watermark_ns_", fusion)
        self.assertIn("hasSourceStampAtOrAfter(source_stamp_ns)", fusion)
        self.assertIn("globalClearEvidenceReady", fusion)
        self.assertIn("rayClearEvidenceReady", fusion)
        self.assertIn("takeDiscardedSourceStampNs", fusion)
        self.assertIn("markObstacleHitEvidenceIncompleteThrough", fusion)
        self.assertIn("clearEvidenceExpired(source_stamp_ns, received_at)", fusion)
        self.assertIn("excludeCurrentObstacleHitsFromRayClears", fusion)
        self.assertIn("ray_clear_evidence_[index].reset()", fusion)
        ray_callback_start = fusion.index("void clearingRayCallback(")
        ray_callback_end = fusion.index("bool processClearingRayFrame(", ray_callback_start)
        ray_callback = fusion[ray_callback_start:ray_callback_end]
        self.assertNotIn("recordObstacleHitEvidence", ray_callback)
        self.assertIn("processClearingRayFrame", ray_callback)
        self.assertIn("pending_obstacle_ray_frames_.enqueue", ray_callback)
        retry_start = fusion.index("void retryPendingGlobalClearEvents(")
        retry_end = fusion.index("bool applyGlobalClearEvent(", retry_start)
        retry_callback = fusion[retry_start:retry_end]
        self.assertIn("pending_obstacle_hit_frames_.retryAvailable", retry_callback)
        self.assertIn("pending_obstacle_ray_frames_.retryAvailable", retry_callback)
        self.assertIn("pending_global_clear_events_.retryAvailableNewestFirst", retry_callback)
        self.assertLess(
            retry_callback.index("pending_obstacle_hit_frames_.retryAvailable"),
            retry_callback.index("pending_obstacle_ray_frames_.retryAvailable"))
        self.assertLess(
            retry_callback.index("pending_obstacle_ray_frames_.retryAvailable"),
            retry_callback.index("pending_global_clear_events_.retryAvailable"))
        self.assertNotIn("pending_obstacle_hit_frames_.empty()", retry_callback)
        update_policy = (Path(__file__).resolve().parents[1] / "include" / "bringup" /
                         "map_fusion_update.hpp").read_text()
        self.assertIn("confirmed_published_cutoff_ns >= clear_event_stamp_ns", update_policy)
        self.assertIn("obstacle_evidence_watermark_ns >= confirmed_published_cutoff_ns", update_policy)
        filter_source = (Path(__file__).resolve().parents[1] / "src" /
                         "nonground_height_filter.cpp").read_text()
        self.assertIn("ground_veto_fence_service_ = create_service", filter_source)
        self.assertIn("publishGroundVeto(message->header, ground_veto)", filter_source)
        self.assertIn(
            "ground->header.stamp.sec != ground_veto->header.stamp.sec", fusion)

    def test_terrain_converter_accepts_compressed_pcd(self):
        converter = (Path(__file__).resolve().parents[1] / "src" /
                     "pcd_to_terrain_map.cpp").read_text()
        self.assertIn('header.data == "binary_compressed"', converter)
        self.assertIn("lzfDecompress", converter)
        self.assertIn('WIDTH*HEIGHT', converter)
        self.assertIn('std::optional<BinaryCompressedPayload>', converter)
        self.assertIn('map.add("class", 0.0F)', converter)
        self.assertIn('map.add("clearance",', converter)
        self.assertIn('free_thresh: 0.196', converter)

    def test_terrain_converter_does_not_free_rejected_surfaces(self):
        converter = (Path(__file__).resolve().parents[1] / "src" /
                     "pcd_to_terrain_map.cpp").read_text()
        self.assertIn("const bool has_valid_ground = ground_valid[flat]", converter)
        self.assertIn("TerrainCellClass::Unknown", converter)
        self.assertIn("TerrainCellClass::Obstacle", converter)
        self.assertIn("if (has_valid_ground)", converter)
        self.assertIn("fastdem::applyInpainting(map, 5, 3, true)", converter)
        self.assertNotIn("inferGroundGap", converter)
        self.assertNotIn('map.get("class")(row, col) = 3.0F', converter)
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("robot_height", default_value="1.20")', launch)
        self.assertIn('DeclareLaunchArgument("clearance_margin", default_value="0.10")', launch)
        self.assertIn(
            "global_costmap.global_costmap.ros__parameters.static_terrain_layer.robot_height",
            launch)
        self.assertIn('"robot_height": ParameterValue(robot_height, value_type=float)', launch)
        self.assertIn(
            '"clearance_margin": ParameterValue(clearance_margin, value_type=float)', launch)
        height_filter = (Path(__file__).resolve().parents[1] / "src" /
                         "nonground_height_filter.cpp").read_text()
        self.assertIn('"no_elevation_z_max", robot_height_ + clearance_margin_', height_filter)

    def test_map_cache_and_saved_snapshots_are_versioned_by_ground_semantics(self):
        script = (Path(__file__).resolve().parents[3] / "tool" /
                  "start_map_navigation.sh").read_text()
        self.assertIn('MAP_SEMANTICS_VERSION="measured_ground_clearance_v2"', script)
        self.assertIn("map_semantics=%q", script)
        self.assertIn(
            'saved_map_version="${saved_map_namespace}/${map_source_id}/latest"',
            script)
        self.assertIn('required_clearance_m=%q', script)
        self.assertIn('--required-clearance-m=${required_clearance_m}', script)
        self.assertIn('"${required_clearance_m}" | sha256sum', script)
        self.assertIn('saved_map_namespace="${SAVED_MAP_ROOT}/${MAP_SEMANTICS_VERSION}/${clearance_id}"', script)
        self.assertIn('"saved_map_root:=${saved_map_namespace}"', script)
        self.assertIn('"robot_height:=${ROBOT_HEIGHT}"', script)
        self.assertIn('"clearance_margin:=${CLEARANCE_MARGIN}"', script)
        converter = (Path(__file__).resolve().parents[1] / "src" /
                     "pcd_to_terrain_map.cpp").read_text()
        self.assertIn('double required_clearance_m{1.30}', converter)
        self.assertIn('static_cast<float>(options.required_clearance_m)', converter)
        fusion = (Path(__file__).resolve().parents[1] / "src" /
                  "global_map_fusion_node.cpp").read_text()
        self.assertIn("raw_class > 2.5F", fusion)
        self.assertIn("raw_source > 1.5F", fusion)
        terrain_layer = (Path(__file__).resolve().parents[2] / "nav2" /
                         "nav2_terrain_layer" / "src" / "static_terrain_layer.cpp").read_text()
        self.assertIn("const bool unknown_ground", terrain_layer)
        self.assertNotIn("inferredGroundCost", terrain_layer)

    def test_navigation_map_snapshot_service_is_reachable_from_panel(self):
        package = Path(__file__).resolve().parents[1]
        launch = (package / "launch" / "map_navigation.launch.py").read_text()
        panel = (package / "src" / "navigation_status_panel.cpp").read_text()
        cmake = (package / "CMakeLists.txt").read_text()
        self.assertIn('executable="map_snapshot_saver_node"', launch)
        self.assertIn('"/navigation/save_map"', panel)
        self.assertIn("保存融合地图 (Ctrl+R)", panel)
        self.assertIn("add_executable(map_snapshot_saver_node", cmake)

    def test_nonground_filter_publishes_temporally_filtered_empty_frames(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "nonground_height_filter.cpp").read_text()
        self.assertIn("if (input_points.empty())", source)
        self.assertIn("makeTemporallyConfirmedCloud(", source)
        self.assertIn("makeTrackingUpdateCloud(", source)
        self.assertIn('"operation", 1, sensor_msgs::msg::PointField::UINT8', source)
        self.assertIn("global_blocking_publisher_->publish(makeCloud(header, {}));", source)
        self.assertIn("publishGroundVeto(header, blocking);", source)
        self.assertIn("bringup::isNoElevationBlocking(", source)
        self.assertIn('global_clearing_topic", std::string("/obstacles/global_clearing")', source)
        self.assertIn("global_clearing_publisher_->publish(global_clearing);", source)
        self.assertIn("global_clearing_publisher_->publish(local_clearing);", source)
        self.assertIn("takeRayClearedPoints()", source)
        self.assertNotIn("if (!blocking.empty()) blocking_publisher_", source)
        self.assertIn("rclcpp::KeepLast(10)).reliable()", source)

    def test_elevation_fallback_vetoes_global_rays_with_raw_blockers(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "nonground_height_filter.cpp").read_text()
        fusion = (Path(__file__).resolve().parents[1] / "src" /
                  "global_map_fusion_node.cpp").read_text()
        fallback_start = source.index("void publishWithoutGround(")
        fallback_end = source.index("void cloudCallback(", fallback_start)
        fallback = source[fallback_start:fallback_end]
        self.assertIn("bringup::isNoElevationBlocking(", fallback)
        self.assertIn("publishGroundVeto(header, blocking);", fallback)
        self.assertIn("obstacle_sub_.subscribe(this, ground_veto_topic_", fusion)
        self.assertIn("mapPoints(message, map_from_obstacles, false)", fusion)
        self.assertIn('"global_obstacle_max_range": 10.0',
                      (Path(__file__).resolve().parents[1] / "launch" /
                       "map_navigation.launch.py").read_text())
        self.assertIn('"raytrace_max_range": 20.0',
                      (Path(__file__).resolve().parents[1] / "launch" /
                       "map_navigation.launch.py").read_text())

    def test_no_elevation_fallback_only_enters_no_map_global_costmap(self):
        package = Path(__file__).resolve().parents[1]
        source = (package / "src" / "nonground_height_filter.cpp").read_text()
        fallback_start = source.index("void publishWithoutGround(")
        fallback_end = source.index("void cloudCallback(", fallback_start)
        fallback = source[fallback_start:fallback_end]
        valid_start = source.index("std::size_t no_elev_dropped = 0;")
        valid_end = source.index("PointCloud local_clearing;", valid_start)
        valid_ground = source[valid_start:valid_end]
        launch = (package / "launch" / "nav2_theta_teb.launch.py").read_text()
        map_launch = (package / "launch" / "map_navigation.launch.py").read_text()
        config = yaml.safe_load((package / "config" / "nav2_theta_teb.yaml").read_text())
        global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]

        self.assertIn('"global_costmap_updates_topic": "/obstacles/global_costmap_updates"',
                      launch)
        self.assertNotIn("global_costmap_updates_topic", map_launch)
        self.assertEqual(global_costmap["width"], 15)
        self.assertEqual(local_costmap["width"], 6)
        self.assertIn("bringup::isNoElevationBlocking(", fallback)
        self.assertIn("global_costmap_temporal_obstacle_grid_", fallback)
        self.assertIn("header, blocking, ray_cloud", fallback)
        self.assertIn("global_costmap_temporal_obstacle_grid_, true, false, true", fallback)
        self.assertNotIn("makeTrackingUpdateCloud(header, {}, {})", fallback)
        self.assertNotIn("global_temporal_obstacle_grid_", fallback)
        self.assertIn("global_blocking_publisher_->publish(makeCloud(header, {}));", fallback)
        self.assertIn("if (has_ground_elevation &&", valid_ground)
        self.assertIn("global_blocking.push_back(input);", valid_ground)
        self.assertIn("header, global_blocking, ray_cloud", source)

    def test_navigation_ready_does_not_depend_on_background_query(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "navigation_gate_node.cpp").read_text()
        self.assertIn("pending_command_ >= 0);", source)
        self.assertIn("Navigation readiness changed:", source)

    def test_map_navigation_waits_for_slow_lifecycle_startup(self):
        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        self.assertIn('"lifecycle_request_timeout", default_value="10.0"', launch)

    def test_progress_checker_counts_rotation(self):
        # Narrow-passage manoeuvres are mostly rotation at max_vel_theta = 0.30 rad/s, which
        # a translation-only checker reports as no progress.
        config = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        checker = config["controller_server"]["ros__parameters"]["progress_checker"]
        self.assertEqual(checker["plugin"], "nav2_controller::PoseProgressChecker")
        self.assertGreater(checker["required_movement_angle"], 0.0)

    def test_disabled_teb_feedback_does_not_create_its_publisher(self):
        source = (Path(__file__).resolve().parents[2] / "nav2" /
                  "teb_local_planner" / "src" / "visualization.cpp").read_text()
        publisher = source.index(
            'create_publisher<teb_msgs::msg::FeedbackMsg>("teb_feedback", 1)')
        self.assertGreater(
            source.rfind("if (cfg_->trajectory.publish_feedback)", 0, publisher), -1)
        for lifecycle_method in ("on_activate()", "on_deactivate()"):
            body = source.split(f"TebVisualization::{lifecycle_method}", 1)[1]
            self.assertIn("if (feedback_pub_)", body.split("}", 1)[0])
        self.assertGreaterEqual(source.count("if (!feedback_pub_)"), 2)

    def test_teb_inflation_stays_below_narrow_passage_clearance(self):
        # Measured clearance in a 1.0--1.5 m passage is 0.23--0.54 m per side; an
        # inflation_dist above that keeps both walls pushing at once and the robot stalls.
        teb = (yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
               ["controller_server"]["ros__parameters"]["FollowPath"])
        self.assertGreater(teb["inflation_dist"], teb["min_obstacle_dist"])
        self.assertLessEqual(teb["inflation_dist"], 0.23)

    def test_teb_allows_bounded_reverse_escape(self):
        teb = (yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
               ["controller_server"]["ros__parameters"]["FollowPath"])
        self.assertTrue(teb["allow_init_with_backwards_motion"])
        # Still prefers forward motion, but not so strongly that a wedged robot cannot back out.
        self.assertGreater(teb["weight_kinematics_forward_drive"], 1.0)
        self.assertLessEqual(teb["weight_kinematics_forward_drive"], 30.0)
        self.assertLess(teb["max_vel_x_backwards"], teb["max_vel_x"])

    def test_both_navigation_configs_use_the_guarded_recovery_tree(self):
        trees = Path(__file__).resolve().parents[1] / "behavior_trees"
        expected_trees = {
            "nav2_theta_teb.yaml": "navigate_to_pose_guarded_recovery.xml",
            "nav2_map_navigation.yaml": "navigate_to_pose_map_guarded_recovery.xml",
        }
        for name in expected_trees.values():
            self.assertTrue((trees / name).is_file())
        self.assertFalse((trees / "navigate_to_pose_no_backup.xml").exists())
        for filename, tree_name in expected_trees.items():
            with self.subTest(filename=filename):
                config = yaml.safe_load((CONFIG / filename).read_text())
                params = config["bt_navigator"]["ros__parameters"]
                self.assertTrue(params["default_nav_to_pose_bt_xml"].endswith(tree_name))
                # The BackUp node and behaviour plugin must both be registered.
                self.assertIn("nav2_back_up_action_bt_node", params["plugin_lib_names"])
                behaviours = config["behavior_server"]["ros__parameters"]
                self.assertIn("backup", behaviours["behavior_plugins"])

    def test_guarded_recovery_tree_backs_up_before_goal_cost_gated_actions(self):
        tree = (Path(__file__).resolve().parents[1] / "behavior_trees" /
                "navigate_to_pose_guarded_recovery.xml").read_text()
        root = ET.fromstring(tree)
        backups = root.findall(".//RoundRobin/BackUp")
        self.assertEqual(len(backups), 1)
        self.assertEqual(float(backups[0].get("backup_dist")), 0.2)
        self.assertEqual(float(backups[0].get("backup_speed")), 0.08)
        self.assertEqual(float(backups[0].get("time_allowance")), 8.0)
        entry_gate = root.find(
            ".//ReactiveSequence[@name='RecoveryForNonlethalGoal']/GoalCostSafe")
        self.assertIsNotNone(entry_gate)
        self.assertEqual(entry_gate.get("max_cost_exclusive"), "254")
        self.assertEqual(entry_gate.get("require_new_snapshot"), "false")
        order = [child.get("name") for child in root.findall(".//RoundRobin/Sequence")]
        self.assertLess(order.index("ClearingActionsForSafeGoal"), order.index("SpinForSafeGoal"))
        fallback = root.find(".//ReactiveFallback[@name='RecoveryFallback']")
        self.assertIsNotNone(fallback)
        self.assertEqual(list(fallback)[1].tag, "ReactiveSequence")

        map_tree = (Path(__file__).resolve().parents[1] / "behavior_trees" /
                    "navigate_to_pose_map_guarded_recovery.xml")
        map_root = ET.parse(map_tree).getroot()
        map_backups = map_root.findall(".//RoundRobin/BackUp")
        self.assertEqual(len(map_backups), 0)
        map_recovery = map_root.find(".//FollowPathRetry")
        self.assertEqual([child.tag for child in map_recovery], ["FollowPath", "PathCollisionFree"])
        self.assertFalse(map_root.findall(".//RecoveryNode"))

    def test_map_navigation_isolates_all_nav2_action_endpoints(self):
        launch_path = (Path(__file__).resolve().parents[1] / "launch" /
                       "nav2_navigation_launch.py")
        tree = ast.parse(launch_path.read_text())
        endpoint_maps = [
            ast.literal_eval(node.value)
            for node in ast.walk(tree)
            if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and
                target.id == "navigate_to_pose_remappings" for target in node.targets)
        ]
        self.assertEqual(len(endpoint_maps), 1)
        endpoint_map = dict(endpoint_maps[0])
        self.assertEqual(len(endpoint_map), 5)
        for endpoint in ("send_goal", "get_result", "cancel_goal", "feedback", "status"):
            self.assertEqual(
                endpoint_map[f"/navigate_to_pose/_action/{endpoint}"],
                f"/navigate_to_pose_internal/_action/{endpoint}")

        navigator_count = 0
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            keywords = {item.arg: item.value for item in node.keywords}
            name = keywords.get("name")
            if not isinstance(name, ast.Constant) or name.value != "bt_navigator":
                continue
            navigator_count += 1
            remapping = keywords["remappings"]
            self.assertIsInstance(remapping, ast.BinOp)
            self.assertIsInstance(remapping.right, ast.Name)
            self.assertEqual(remapping.right.id, "navigate_to_pose_remappings")
        self.assertEqual(navigator_count, 2)

    def test_map_navigation_routes_behavior_commands_through_velocity_smoother(self):
        launch_dir = Path(__file__).resolve().parents[1] / "launch"
        map_launch = (launch_dir / "map_navigation.launch.py").read_text()
        self.assertIn('"nav2_navigation_launch.py"', map_launch)

        nav_launch = ast.parse((launch_dir / "nav2_navigation_launch.py").read_text())
        behavior_nodes = []
        for node in ast.walk(nav_launch):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name) or node.func.id not in ("Node", "ComposableNode"):
                continue
            keywords = {item.arg: item.value for item in node.keywords}
            name = keywords.get("name")
            if isinstance(name, ast.Constant) and name.value == "behavior_server":
                behavior_nodes.append(keywords.get("remappings"))

        self.assertEqual(len(behavior_nodes), 2)
        for remappings in behavior_nodes:
            self.assertIsInstance(remappings, ast.BinOp)
            self.assertIsInstance(remappings.op, ast.Add)
            self.assertIsInstance(remappings.right, ast.List)
            self.assertIn(
                ("cmd_vel", "cmd_vel_nav"),
                [tuple(item.value for item in entry.elts)
                 for entry in remappings.right.elts if isinstance(entry, ast.Tuple)],
            )

    def test_nonground_filter_rejects_implausibly_high_ground_reference(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "nonground_height_filter.cpp").read_text()
        self.assertIn('declare_parameter("ground_reference_max_slope"', source)
        self.assertIn("implausible_ground", source)
        self.assertIn("has_ground_elevation = false;", source)

    def test_map_navigation_temporal_obstacle_and_terrain_parameters(self):
        mapping = yaml.safe_load((CONFIG / "nav2_map_navigation.yaml").read_text())
        local = mapping["local_costmap"]["local_costmap"]["ros__parameters"]
        obstacle = local["temporal_obstacle_layer"]
        terrain = local["terrain_layer"]
        self.assertEqual(obstacle["observation_topic"], "/obstacles/blocking")
        self.assertEqual(obstacle["observation_timeout"], 1.0)
        self.assertEqual(terrain["map_timeout"], 1.5)
        self.assertEqual(terrain["residual_step_soft_threshold"], 0.02)
        self.assertEqual(terrain["lethal_clear_frames"], 2)
        self.assertEqual(terrain["residual_step_lethal_threshold"], 0.05)
        self.assertTrue(terrain["residual_step_hard_constraint"])

        fastdem = yaml.safe_load((CONFIG / "fastdem_local.yaml").read_text())
        self.assertEqual(fastdem["topics"]["max_map_age"], 1.5)

        launch = (Path(__file__).resolve().parents[1] / "launch" /
                  "map_navigation.launch.py").read_text()
        for setting in (
                '"map_timeout": 1.5', '"terrain_max_age": 1.5',
                '"temporal_obstacle_filter_enabled": True',
                '"tracking_frame": "odom"', '"raytrace_topic": "/cloud_registered_body"',
                '"obstacle_confirmation_window_frames": 5',
                '"obstacle_confirmation_hits": 2', '"obstacle_clear_ray_frames": 2',
                '"obstacle_max_unobserved_frames": 20',
                '"obstacle_raytrace_height_tolerance": 0.25'):
            with self.subTest(setting=setting):
                self.assertIn(setting, launch)

    def test_static_pcd_clearance_uses_fastlio_pointwise_ray_origins(self):
        bringup_root = Path(__file__).resolve().parents[1]
        launch = (bringup_root / "launch" / "map_navigation.launch.py").read_text()
        publisher = (bringup_root / "src" / "static_terrain_map_publisher.cpp").read_text()
        fastlio_root = Path(__file__).resolve().parents[2] / "FAST_LIO_ROS2"
        imu_processing = (fastlio_root / "src" / "IMU_Processing.hpp").read_text()
        laser_mapping = (fastlio_root / "src" / "laserMapping.cpp").read_text()

        self.assertNotIn("clearance_ray_origin", launch)
        self.assertNotIn("clearance_ray_origin_in_body", publisher)
        self.assertIn('"/cloud_registered_body_clearance"', laser_mapping)
        self.assertIn('"time_from_scan_end"', laser_mapping)
        self.assertIn("pointBodyMotionInScanEnd", imu_processing)
        self.assertIn("state_point.offset_T_L_I", laser_mapping)
        self.assertIn("ray_origin_x", publisher)


if __name__ == "__main__":
    unittest.main()
