#!/usr/bin/env python3
import argparse
import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from analyze_pcd_terrain_map import SWEEP_CASES, SWEEP_CSV_FIELDS, SWEEP_STAT_FIELDS, run_sweep


class PcdTerrainMapSweepTest(unittest.TestCase):
    def run_mocked_sweep(self, successful_indices):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            converter = root / "converter"
            converter.write_text("mock converter\n", encoding="utf-8")
            converter.chmod(0o755)
            pcd = root / "input.pcd"
            pcd.write_text("mock PCD\n", encoding="utf-8")
            output_root = root / "sweep"

            call_index = 0

            def mock_run(command, **_kwargs):
                nonlocal call_index
                current_index = call_index
                call_index += 1
                if current_index in successful_indices:
                    Path(command[3]).with_suffix(".pgm").write_text(
                        "P2\n2 1\n255\n254 0\n", encoding="ascii")
                    return SimpleNamespace(returncode=0, stdout="", stderr="")
                return SimpleNamespace(
                    returncode=2, stdout="", stderr=f"synthetic failure {current_index}")

            args = argparse.Namespace(
                converter=converter, pcd=pcd, output_root=output_root)
            with patch("analyze_pcd_terrain_map.subprocess.run", side_effect=mock_run):
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                    io.StringIO()
                ):
                    return_code = run_sweep(args)

            with (output_root / "map_sweep.csv").open(
                newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                fieldnames = reader.fieldnames
                rows = list(reader)
            return return_code, fieldnames, rows

    def assert_complete_report(self, successful_indices, expected_return_code):
        return_code, fieldnames, rows = self.run_mocked_sweep(successful_indices)

        self.assertEqual(return_code, expected_return_code)
        self.assertEqual(fieldnames, list(SWEEP_CSV_FIELDS))
        self.assertEqual(len(rows), len(SWEEP_CASES))
        for index, row in enumerate(rows):
            if index in successful_indices:
                self.assertEqual(row["return_code"], "0")
                self.assertEqual(row["width"], "2")
                self.assertEqual(row["free_cells"], "1")
                self.assertEqual(row["occupied_cells"], "1")
                self.assertEqual(row["error"], "")
            else:
                self.assertEqual(row["return_code"], "2")
                self.assertEqual(row["error"], f"synthetic failure {index}")
                self.assertTrue(all(row[field] == "" for field in SWEEP_STAT_FIELDS))

    def test_all_success(self):
        self.assert_complete_report(set(range(len(SWEEP_CASES))), 0)

    def test_all_failure(self):
        self.assert_complete_report(set(), 1)

    def test_failure_then_success_keeps_later_statistics(self):
        self.assert_complete_report(set(range(1, len(SWEEP_CASES))), 1)

    def test_success_then_failure_keeps_earlier_statistics(self):
        self.assert_complete_report({0}, 1)


if __name__ == "__main__":
    unittest.main()
