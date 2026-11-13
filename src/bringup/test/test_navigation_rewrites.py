"""Expand navigation parameters without starting launch actions or ROS nodes."""
import ast
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration


PACKAGE = Path(__file__).resolve().parents[1]


class NavigationRewritesTest(unittest.TestCase):
    def test_parameter_expansion_for_both_launches_and_chassis_models(self):
        profiles = yaml.safe_load((PACKAGE / 'config/chassis_models.yaml').read_text())
        for stem in ('map_navigation', 'nav2_theta_teb'):
            launch_path = PACKAGE / 'launch' / (stem + '.launch.py')
            spec = importlib.util.spec_from_file_location(stem, launch_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            tree = ast.parse(launch_path.read_text())
            call = next(node for node in ast.walk(tree)
                        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                        and node.func.id == 'RewrittenYaml')
            original = yaml.safe_load((PACKAGE / 'config' / ('nav2_map_navigation.yaml'
                                      if stem == 'map_navigation' else stem + '.yaml')).read_text())
            # Custom longitudinal/yaw limits must survive lateral overrides.
            limits = original['velocity_smoother']['ros__parameters']
            limits['max_velocity'] = [0.23, 0.07, 0.27]
            for model in ('two_wheel', 'omni'):
                with self.subTest(launch=stem, model=model), tempfile.TemporaryDirectory() as tmp:
                    source = Path(tmp) / 'params.yaml'
                    source.write_text(yaml.safe_dump(original))
                    context = LaunchContext()
                    context.launch_configurations['chassis_model'] = model
                    scope = dict(vars(module))
                    for node in ast.walk(call):
                        if isinstance(node, ast.Name) and node.id not in scope:
                            scope[node.id] = 'false'
                    scope.update(params_file=str(source), robot_height='1.2',
                                 clearance_margin='0.1', terrain_map_file='terrain.npz',
                                 robot_footprint=json.dumps(profiles[model]['footprint']),
                                 chassis_model=LaunchConfiguration('chassis_model'),
                                 omni_lateral_speed='0.15' if model == 'omni' else '0.0',
                                 omni_lateral_accel='0.30' if model == 'omni' else '0.0')
                    rewrite = eval(compile(ast.Expression(call), str(launch_path), 'eval'), scope)
                    result_path = Path(rewrite.perform(context))
                    try:
                        result = yaml.safe_load(result_path.read_text())
                    finally:
                        result_path.unlink()
                    smoother = result['velocity_smoother']['ros__parameters']
                    lateral = (0.15, -0.15, 0.30, -0.30) if model == 'omni' else (0., 0., 0., 0.)
                    for key, y in zip(('max_velocity', 'min_velocity', 'max_accel', 'max_decel'), lateral):
                        expected = list(limits[key])
                        expected[1] = y
                        self.assertEqual(smoother[key], expected)
                        self.assertTrue(all(isinstance(value, float) for value in smoother[key]))
                    controller = result['controller_server']['ros__parameters']['FollowPath']
                    self.assertEqual(json.loads(controller['footprint_model.vertices']),
                                     profiles[model]['footprint'])
                    self.assertNotIn('footprint_model', controller)
                    for costmap in ('global_costmap', 'local_costmap'):
                        self.assertEqual(json.loads(result[costmap][costmap]['ros__parameters']['footprint']),
                                         profiles[model]['footprint'])

    def test_rewrite_does_not_mutate_the_path(self):
        path = PACKAGE / 'launch/map_navigation.launch.py'
        spec = importlib.util.spec_from_file_location('map_rewrites', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rewrite = module.RewrittenYaml(source_file='unused', param_rewrites={})
        keys = ['root', 'values', '1']
        data = {'root': {'values': [1., 2., 3.]}}
        self.assertEqual(rewrite.updateYamlPathVals(data, keys, 4.),
                         {'root': {'values': [1., 4., 3.]}})
        self.assertEqual(keys, ['root', 'values', '1'])


if __name__ == '__main__':
    unittest.main()
