import importlib.util
import math
from pathlib import Path
import struct
import tempfile
import unittest

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pcd_to_occupancy_grid.py"
SPEC = importlib.util.spec_from_file_location("pcd_to_occupancy_grid", SCRIPT)
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


class MapConversionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "map.yaml"

    def convert(self, points, **overrides):
        parameters = dict(resolution=1.0, min_z=0.1, max_z=1.5,
                          occupied_radius=0.0, ground_min_z=-0.3, ground_max_z=-0.1)
        parameters.update(overrides)
        return converter.write_map(points, self.output, **parameters)

    def pixels(self):
        return self.output.with_suffix(".pgm").read_bytes().split(b"\n", 3)[3]

    def test_unobserved_cells_remain_unknown(self):
        self.convert([(0, 0, -0.2), (2, 0, -0.2), (2, 0, 0.5)])
        self.assertEqual(self.pixels(), bytes([254, 205, 0]))

    def test_ground_only_map_is_valid(self):
        self.assertEqual(self.convert([(0, 0, -0.2)]), (1, 1, 0, 0))
        self.assertEqual(self.pixels(), bytes([254]))

    def test_obstacle_radius_overrides_ground(self):
        self.convert([(0, 0, -0.2), (1, 0, -0.2), (2, 0, -0.2), (1, 0, 0.5)],
                     occupied_radius=1.0)
        self.assertEqual(self.pixels(), bytes([0, 0, 0]))

    def test_image_rows_and_negative_origin(self):
        self.convert([(-1, -1, -0.2), (0, 0, 0.5)])
        self.assertEqual(self.pixels(), bytes([205, 0, 254, 205]))
        self.assertEqual(yaml.safe_load(self.output.read_text())["origin"], [-1, -1, 0])

    def test_invalid_points_are_ignored(self):
        self.convert([(0, 0, -0.2), (math.nan, 0, 0.5), (0, math.inf, -0.2),
                      (0, 0, math.nan)])
        self.assertEqual(self.pixels(), bytes([254]))

    def test_no_ground_fails_without_output(self):
        for points in ([], [(0, 0, 0.5)], [(math.nan, 0, -0.2)]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.convert(points)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.output.with_suffix(".pgm").exists())

    def test_invalid_parameters_are_rejected(self):
        for overrides in ({"resolution": 0}, {"resolution": math.nan},
                          {"occupied_radius": math.inf}, {"occupied_radius": -1},
                          {"min_z": 2}, {"ground_max_z": 0.1},
                          {"ground_min_z": 0}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.convert([(0, 0, -0.2)], **overrides)

    def test_oversized_map_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "25 million"):
            self.convert([(0, 0, -0.2), (10000, 10000, -0.2)])

    def test_yaml_image_name_is_escaped(self):
        self.output = self.output.with_name("map: #1.yaml")
        self.convert([(0, 0, -0.2)])
        self.assertEqual(yaml.safe_load(self.output.read_text())["image"], "map: #1.pgm")

    def test_non_yaml_output_is_rejected(self):
        self.output = self.output.with_suffix(".pgm")
        with self.assertRaises(ValueError):
            self.convert([(0, 0, -0.2)])
        self.assertFalse(self.output.exists())

    def read_pcd(self, header, payload):
        path = Path(self.temp.name) / "cloud.pcd"
        path.write_bytes(header.encode("ascii") + payload)
        return converter.read_pcd(path)

    def test_ascii_points_without_dimensions(self):
        self.assertEqual(self.read_pcd(
            "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 1\nDATA ascii\n",
            b"1 2 3\n"), [(1, 2, 3)])

    def test_binary_field_offsets_and_dimension_fallback(self):
        self.assertEqual(self.read_pcd(
            "FIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\n"
            "WIDTH 1\nHEIGHT 1\nDATA binary\n",
            struct.pack("<ffff", 99, 3, 1, 2)), [(1, 2, 3)])

    def test_truncated_ascii_and_binary_are_rejected(self):
        for data_format, payload in (("ascii", b"1 2 3\n"),
                                     ("binary", struct.pack("<fff", 1, 2, 3))):
            with self.subTest(data_format=data_format), self.assertRaises(ValueError):
                self.read_pcd("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                              f"POINTS 2\nDATA {data_format}\n", payload)

    def test_malformed_header_is_rejected(self):
        for fields in ("SIZE 4 4 4\n", "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1\n"):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.read_pcd(f"FIELDS x y z\n{fields}POINTS 1\nDATA ascii\n", b"1 2 3\n")


if __name__ == "__main__":
    unittest.main()
