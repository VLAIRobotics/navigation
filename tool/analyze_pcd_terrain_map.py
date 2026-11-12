#!/usr/bin/env python3
"""Measure PGM classes and sweep offline PCD-to-terrain-map parameters."""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import tempfile
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class SweepCase:
    name: str
    analysis_radius: float = 0.30
    min_valid_neighbors: int = 16
    min_ground_neighbors: int = 1
    min_plane_points: int = 6
    min_ground_component_cells: int = 10
    min_observations: int = 3
    max_ground_normal_angle: int = 35
    ground_component_height_band: float = 0.35


SWEEP_CASES = (
    SweepCase("baseline"),
    SweepCase("valid_neighbors_12", min_valid_neighbors=12),
    SweepCase("valid_neighbors_8", min_valid_neighbors=8),
    SweepCase("valid_neighbors_6", min_valid_neighbors=6),
    SweepCase("radius_0.20_neighbors_6", analysis_radius=0.20, min_valid_neighbors=6),
    SweepCase("ground_neighbors_0", min_valid_neighbors=6, min_ground_neighbors=0),
    SweepCase(
        "ground_neighbors_0_plane_points_4", min_valid_neighbors=6,
        min_ground_neighbors=0, min_plane_points=4),
    SweepCase(
        "ground_neighbors_0_plane_points_3", min_valid_neighbors=6,
        min_ground_neighbors=0, min_plane_points=3),
    SweepCase(
        "plane_points_4_component_cells_1", min_valid_neighbors=6,
        min_ground_neighbors=0, min_plane_points=4,
        min_ground_component_cells=1),
    SweepCase("observations_2", min_valid_neighbors=6, min_observations=2),
    SweepCase("observations_1", min_valid_neighbors=6, min_observations=1),
    SweepCase("normal_angle_45", min_valid_neighbors=6, max_ground_normal_angle=45),
    SweepCase("normal_angle_60", min_valid_neighbors=6, max_ground_normal_angle=60),
    SweepCase("normal_angle_80", min_valid_neighbors=6, max_ground_normal_angle=80),
    SweepCase(
        "ground_height_band_0.50", min_valid_neighbors=6,
        ground_component_height_band=0.50),
    SweepCase(
        "ground_height_band_0.75", min_valid_neighbors=6,
        ground_component_height_band=0.75),
    SweepCase(
        "ground_height_band_1.00", min_valid_neighbors=6,
        ground_component_height_band=1.00),
)

SWEEP_STAT_FIELDS = (
    "width", "height", "total_cells", "free_cells", "occupied_cells",
    "unknown_cells", "free_pct", "occupied_pct", "unknown_pct",
)
SWEEP_CSV_FIELDS = (
    "case", "analysis_radius", "min_valid_neighbors", "min_ground_neighbors",
    "min_plane_points", "min_ground_component_cells", "min_observations",
    "max_ground_normal_angle", "ground_component_height_band", "elapsed_sec",
    "return_code", "map_yaml", *SWEEP_STAT_FIELDS, "error",
)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    """Read P2 or P5 PGM, including comments and multi-line headers."""
    data = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in b" \t\r\n\f\v":
            index += 1
        if index >= len(data):
            raise ValueError(f"incomplete PGM header: {path}")
        if data[index] == ord("#"):
            newline = data.find(b"\n", index)
            if newline < 0:
                raise ValueError(f"unterminated PGM comment: {path}")
            index = newline + 1
            continue
        end = index
        while end < len(data) and data[end] not in b" \t\r\n\f\v#":
            end += 1
        tokens.append(data[index:end])
        index = end

    magic, width_token, height_token, maxval_token = tokens
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"unsupported PGM type {magic!r}: {path}")
    width = int(width_token)
    height = int(height_token)
    maxval = int(maxval_token)
    if width <= 0 or height <= 0 or not 1 <= maxval <= 65535:
        raise ValueError(f"invalid PGM dimensions or maxval: {path}")

    if magic == b"P5":
        # The binary raster begins after the mandatory whitespace separator.
        if index >= len(data) or data[index] not in b" \t\r\n\f\v":
            raise ValueError(f"missing PGM raster separator: {path}")
        if data[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        sample_size = 1 if maxval < 256 else 2
        expected = width * height * sample_size
        raster = data[index:]
        if len(raster) != expected:
            raise ValueError(
                f"PGM raster size mismatch for {path}: expected {expected}, got {len(raster)}")
        if sample_size == 2:
            values = [int.from_bytes(raster[i:i + 2], "big")
                      for i in range(0, len(raster), 2)]
            raster = bytes(min(255, round(value * 255 / maxval)) for value in values)
        elif maxval != 255:
            raster = bytes(min(255, round(value * 255 / maxval)) for value in raster)
        return width, height, raster

    values = [int(token) for token in data[index:].split()]
    if len(values) != width * height:
        raise ValueError(
            f"PGM raster size mismatch for {path}: expected {width * height}, got {len(values)}")
    raster = bytes(min(255, round(value * 255 / maxval)) for value in values)
    return width, height, raster


def summarize_map(path: Path) -> dict[str, int | float | str]:
    width, height, raster = read_pgm(path)
    counts = Counter(raster)
    total = len(raster)
    free = counts[254]
    occupied = counts[0]
    unknown = total - free - occupied
    return {
        "map": str(path),
        "width": width,
        "height": height,
        "total_cells": total,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "free_pct": 100.0 * free / total,
        "occupied_pct": 100.0 * occupied / total,
        "unknown_pct": 100.0 * unknown / total,
        "other_grayscale_values": sum(
            count for value, count in counts.items() if value not in (0, 254)),
    }


def print_summary(label: str, summary: dict[str, int | float | str]) -> None:
    print(
        f"{label}: {summary['width']}x{summary['height']} "
        f"free={summary['free_pct']:.3f}% ({summary['free_cells']}), "
        f"occupied={summary['occupied_pct']:.3f}% ({summary['occupied_cells']}), "
        f"unknown={summary['unknown_pct']:.3f}% ({summary['unknown_cells']})")


def run_sweep(args: argparse.Namespace) -> int:
    converter = args.converter.resolve()
    pcd = args.pcd.resolve()
    if not converter.is_file() or not converter.stat().st_mode & 0o111:
        raise FileNotFoundError(f"converter is missing or not executable: {converter}")
    if not pcd.is_file():
        raise FileNotFoundError(f"PCD file not found: {pcd}")

    output_root = args.output_root
    if output_root is None:
        output_root = Path(tempfile.mkdtemp(prefix="nav2_pcd_map_sweep_"))
    else:
        output_root = output_root.resolve()
        if output_root.exists():
            raise FileExistsError(f"sweep output directory already exists: {output_root}")
        output_root.mkdir(parents=True, exist_ok=False)
    report_path = output_root / "map_sweep.csv"
    rows: list[dict[str, str | int | float]] = []

    print(f"PCD: {pcd}")
    print(f"Converter: {converter}")
    print(f"Sweep outputs: {output_root}")
    print("PGM classes use converter values 254=free, 0=occupied, other=unknown.")

    for case in SWEEP_CASES:
        case_dir = output_root / case.name
        case_dir.mkdir(parents=True, exist_ok=True)
        terrain_path = case_dir / "terrain_map.npz"
        yaml_path = case_dir / "map.yaml"
        command = [
            str(converter), str(pcd), str(terrain_path), str(yaml_path),
            "--resolution=0.10",
            f"--analysis-radius={case.analysis_radius:.2f}",
            f"--min-valid-neighbors={case.min_valid_neighbors}",
            f"--min-ground-neighbors={case.min_ground_neighbors}",
            f"--min-plane-points={case.min_plane_points}",
            f"--min-ground-component-cells={case.min_ground_component_cells}",
            f"--min-observations={case.min_observations}",
            f"--max-ground-normal-angle={case.max_ground_normal_angle}",
            f"--ground-component-height-band={case.ground_component_height_band:.2f}",
        ]
        started = time.monotonic()
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        elapsed = time.monotonic() - started
        row: dict[str, str | int | float] = {
            "case": case.name,
            "analysis_radius": case.analysis_radius,
            "min_valid_neighbors": case.min_valid_neighbors,
            "min_ground_neighbors": case.min_ground_neighbors,
            "min_plane_points": case.min_plane_points,
            "min_ground_component_cells": case.min_ground_component_cells,
            "min_observations": case.min_observations,
            "max_ground_normal_angle": case.max_ground_normal_angle,
            "ground_component_height_band": case.ground_component_height_band,
            "elapsed_sec": round(elapsed, 3),
            "return_code": result.returncode,
            "map_yaml": str(yaml_path),
            **{key: "" for key in SWEEP_STAT_FIELDS},
            "error": "",
        }
        if result.returncode == 0:
            pgm_path = yaml_path.with_suffix(".pgm")
            summary = summarize_map(pgm_path)
            row.update({key: summary[key] for key in SWEEP_STAT_FIELDS})
            print_summary(case.name, summary)
            if result.stdout.strip():
                print(result.stdout.strip().splitlines()[-1])
        else:
            row["error"] = result.stderr.strip().replace("\n", " | ")
            print(f"{case.name}: FAILED ({result.returncode}) {row['error']}", file=sys.stderr)
        rows.append(row)

    with report_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SWEEP_CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"CSV: {report_path}")
    return 1 if any(row["return_code"] != 0 for row in rows) else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map", type=Path, default=REPO_ROOT / "PCD/clean_map_nav/map.pgm",
        help="existing PGM to summarize")
    parser.add_argument("--run-sweep", action="store_true", help="run offline converter variants")
    parser.add_argument(
        "--pcd", type=Path, default=REPO_ROOT / "PCD/clean_map.pcd",
        help="input PCD for the converter sweep")
    parser.add_argument(
        "--converter", type=Path,
        default=REPO_ROOT / "install/bringup/lib/bringup/pcd_to_terrain_map",
        help="built pcd_to_terrain_map executable")
    parser.add_argument(
        "--output-root", type=Path,
        help="sweep output directory (defaults to a new directory under /tmp)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.map.is_file():
        print_summary("existing", summarize_map(args.map))
    else:
        print(f"Existing PGM not found: {args.map}", file=sys.stderr)
        if not args.run_sweep:
            return 1
    if args.run_sweep:
        return run_sweep(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
