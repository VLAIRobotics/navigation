#!/usr/bin/env python3

"""Convert a static XYZ PCD into a Nav2-compatible trinary map.

Requires explicit obstacle and ground height intervals in the PCD frame.
For level-ground maps, only cells containing ground samples are marked free.
Unobserved cells remain unknown and obstacles take precedence over ground.
Sloped or multi-level maps require ground segmentation before conversion.
"""

import argparse
import json
import math
import pathlib
import struct


def read_pcd(path):
    with path.open("rb") as stream:
        header_lines = []
        data_start = None
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header is incomplete")
            header_lines.append(line)
            if line.upper().startswith(b"DATA "):
                data_start = stream.tell()
                break

        header = {}
        for raw_line in header_lines:
            line = raw_line.decode("ascii", errors="strict").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key.upper()] = values

        fields = header.get("FIELDS")
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE")
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(sizes))]
        if (not fields or not types or len(fields) != len(sizes) or
                len(fields) != len(types) or len(fields) != len(counts) or
                any(size <= 0 for size in sizes)):
            raise ValueError("PCD fields, sizes and types do not match")
        if any(count != 1 for count in counts):
            raise ValueError("PCD fields with COUNT > 1 are not supported")
        try:
            x_index, y_index, z_index = (fields.index(name) for name in ("x", "y", "z"))
        except ValueError as error:
            raise ValueError("PCD must contain x, y and z fields") from error

        if "POINTS" in header:
            points = int(header["POINTS"][0])
        elif "WIDTH" in header and "HEIGHT" in header:
            points = int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
        else:
            raise ValueError("PCD must declare POINTS or WIDTH and HEIGHT")
        if points <= 0:
            raise ValueError("PCD must contain at least one point")
        data_format = header.get("DATA", [""])[0].lower()
        stream.seek(data_start)
        if data_format == "ascii":
            result = []
            for _ in range(points):
                values = stream.readline().split()
                if len(values) != len(fields):
                    raise ValueError("PCD ASCII payload is truncated or has invalid fields")
                result.append((float(values[x_index]), float(values[y_index]), float(values[z_index])))
            return result
        if data_format != "binary":
            raise ValueError("only ASCII and binary PCD are supported")

        offsets = []
        offset = 0
        for size, count in zip(sizes, counts):
            offsets.append(offset)
            offset += size * count
        point_step = offset
        raw = stream.read(points * point_step)
        if len(raw) != points * point_step:
            raise ValueError("PCD binary payload is shorter than POINTS")

        def read_float(index, row_offset):
            if types[index] != "F" or sizes[index] != 4:
                raise ValueError("x, y and z must be float32 fields")
            return struct.unpack_from("<f", raw, row_offset + offsets[index])[0]

        return [
            (read_float(x_index, i), read_float(y_index, i), read_float(z_index, i))
            for i in range(0, len(raw), point_step)
        ]


def write_map(points, yaml_path, resolution, min_z, max_z, occupied_radius,
              ground_min_z, ground_max_z):
    if yaml_path.suffix.lower() not in (".yaml", ".yml"):
        raise ValueError("output map path must end in .yaml or .yml")
    if (not all(math.isfinite(value) for value in
                (resolution, min_z, max_z, occupied_radius, ground_min_z, ground_max_z)) or
            resolution <= 0.0 or min_z > max_z or occupied_radius < 0.0 or
            ground_min_z > ground_max_z or ground_max_z >= min_z):
        raise ValueError("invalid map conversion parameters")
    selected = []
    ground = []
    for point in points:
        if not all(math.isfinite(value) for value in point):
            continue
        if min_z <= point[2] <= max_z:
            selected.append(point)
        elif ground_min_z <= point[2] <= ground_max_z:
            ground.append(point)
    if not ground:
        raise ValueError("no finite ground samples remain in the ground height interval")

    extent = ground + selected
    min_x = min(point[0] for point in extent)
    min_y = min(point[1] for point in extent)
    max_x = max(point[0] for point in extent)
    max_y = max(point[1] for point in extent)
    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    width = max(1, math.floor((max_x - origin_x) / resolution) + 1)
    height = max(1, math.floor((max_y - origin_y) / resolution) + 1)
    if width * height > 25_000_000:
        raise ValueError("map exceeds 25 million cells; crop PCD or increase resolution")
    free = {(math.floor((x - origin_x) / resolution),
             math.floor((y - origin_y) / resolution)) for x, y, _ in ground}
    occupied = set()
    radius_cells = int(math.ceil(occupied_radius / resolution))
    for x, y, _ in selected:
        cell_x = math.floor((x - origin_x) / resolution)
        cell_y = math.floor((y - origin_y) / resolution)
        for dx in range(max(-radius_cells, -cell_x), min(radius_cells, width - 1 - cell_x) + 1):
            for dy in range(max(-radius_cells, -cell_y), min(radius_cells, height - 1 - cell_y) + 1):
                if (dx * resolution) ** 2 + (dy * resolution) ** 2 <= occupied_radius ** 2:
                    candidate_x = cell_x + dx
                    candidate_y = cell_y + dy
                    if 0 <= candidate_x < width and 0 <= candidate_y < height:
                        occupied.add((candidate_x, candidate_y))

    pgm_path = yaml_path.with_suffix(".pgm")
    with pgm_path.open("wb") as image:
        image.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for row in range(height - 1, -1, -1):
            image.write(bytes(0 if (column, row) in occupied else
                              254 if (column, row) in free else 205
                              for column in range(width)))

    yaml_path.write_text(
        """image: {image}\nresolution: {resolution:.9g}\norigin: [{origin_x:.9g}, {origin_y:.9g}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n""".format(
            image=json.dumps(pgm_path.name),
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        ),
        encoding="utf-8",
    )
    return width, height, len(occupied), len(selected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=pathlib.Path)
    parser.add_argument("yaml", type=pathlib.Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--min-z", type=float, required=True)
    parser.add_argument("--max-z", type=float, required=True)
    parser.add_argument("--ground-min-z", type=float, required=True,
                        help="minimum observed ground height in PCD frame (level ground only)")
    parser.add_argument("--ground-max-z", type=float, required=True,
                        help="maximum observed ground height; must be below --min-z")
    parser.add_argument("--occupied-radius", type=float, default=0.10)
    args = parser.parse_args()
    points = read_pcd(args.pcd)
    width, height, occupied, selected = write_map(
        points, args.yaml, args.resolution, args.min_z, args.max_z, args.occupied_radius,
        args.ground_min_z, args.ground_max_z)
    print(
        f"wrote {args.yaml} and {args.yaml.with_suffix('.pgm')} "
        f"({width}x{height}, {occupied} occupied cells, {selected} selected points)"
    )


if __name__ == "__main__":
    main()
