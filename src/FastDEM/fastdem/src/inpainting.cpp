// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include "fastdem/postprocess/inpainting.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <utility>
#include <vector>

namespace fastdem {

void applyInpainting(ElevationMap& map, int max_iterations,
                     int min_valid_neighbors, bool inplace) {
  applyInpainting(map, max_iterations, min_valid_neighbors, inplace,
                  /*max_hole_cells=*/9);
}

void applyInpainting(ElevationMap& map, int max_iterations,
                     int min_valid_neighbors, bool inplace,
                     int max_hole_cells) {
  max_iterations = std::clamp(max_iterations, 1, 32);
  min_valid_neighbors = std::clamp(min_valid_neighbors, 1, 8);
  max_hole_cells = std::clamp(max_hole_cells, 1, 4096);

  const auto size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);
  if (rows == 0 || cols == 0) return;

  // nanoGrid stores layer data in a circular buffer: buffer index
  // b = (logical index l + startIndex) % size. World adjacency maps to
  // logical-coordinate adjacency. Working in logical coordinates keeps hole
  // classification and boundary rejection correct for any rolling offset.
  const auto start = map.getStartIndex();
  const auto buffer_row = [&](int logical_row) {
    return (start(0) + logical_row) % rows;
  };
  const auto buffer_col = [&](int logical_col) {
    return (start(1) + logical_col) % cols;
  };

  const auto& elevation = map.get(layer::elevation);
  const Eigen::MatrixXf observed = elevation.array().isFinite().cast<float>();
  Eigen::MatrixXf result = elevation;
  Eigen::MatrixXf mask = Eigen::MatrixXf::Constant(rows, cols, NAN);

  for (int logical_row = 0; logical_row < rows; ++logical_row) {
    for (int logical_col = 0; logical_col < cols; ++logical_col) {
      const int br = buffer_row(logical_row);
      const int bc = buffer_col(logical_col);
      if (observed(br, bc) > 0.5f) {
        mask(br, bc) = 0.0f;
      } else {
        result(br, bc) = NAN;
      }
    }
  }

  const auto flat = [cols](int logical_row, int logical_col) {
    return logical_row * cols + logical_col;
  };

  std::vector<bool> visited(static_cast<size_t>(rows * cols), false);
  constexpr int offsets[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                 {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  for (int seed_row = 0; seed_row < rows; ++seed_row) {
    for (int seed_col = 0; seed_col < cols; ++seed_col) {
      const int seed_flat = flat(seed_row, seed_col);
      if (visited[seed_flat]) continue;
      if (observed(buffer_row(seed_row), buffer_col(seed_col)) > 0.5f)
        continue;

      std::deque<std::pair<int, int>> queue;
      std::vector<std::pair<int, int>> component;
      bool touches_boundary = false;
      bool oversized = false;
      queue.emplace_back(seed_row, seed_col);
      visited[seed_flat] = true;

      while (!queue.empty()) {
        const auto [row, col] = queue.front();
        queue.pop_front();
        if (!oversized) {
          component.emplace_back(row, col);
          oversized = component.size() > static_cast<size_t>(max_hole_cells);
          if (oversized) component.clear();
        }
        touches_boundary = touches_boundary || row == 0 || col == 0 ||
                           row == rows - 1 || col == cols - 1;

        for (const auto& offset : offsets) {
          const int next_row = row + offset[0];
          const int next_col = col + offset[1];
          if (next_row < 0 || next_row >= rows || next_col < 0 ||
              next_col >= cols) {
            continue;
          }
          const int next_flat = flat(next_row, next_col);
          if (visited[next_flat]) continue;
          if (observed(buffer_row(next_row), buffer_col(next_col)) > 0.5f)
            continue;
          visited[next_flat] = true;
          queue.emplace_back(next_row, next_col);
        }
      }

      if (touches_boundary || oversized) {
        continue;
      }

      for (const auto& [row, col] : component) {
        float weighted_sum = 0.0f;
        float weight_total = 0.0f;
        int count = 0;
        for (int dr = -max_iterations; dr <= max_iterations; ++dr) {
          for (int dc = -max_iterations; dc <= max_iterations; ++dc) {
            if (dr == 0 && dc == 0) continue;
            const int source_row = row + dr;
            const int source_col = col + dc;
            if (source_row < 0 || source_row >= rows || source_col < 0 ||
                source_col >= cols) {
              continue;
            }
            const int distance = std::max(std::abs(dr), std::abs(dc));
            if (distance > max_iterations) continue;
            if (observed(buffer_row(source_row), buffer_col(source_col)) <
                0.5f)
              continue;
            const float value =
                elevation(buffer_row(source_row), buffer_col(source_col));
            if (!std::isfinite(value)) continue;
            const float weight = 1.0f / static_cast<float>(distance);
            weighted_sum += value * weight;
            weight_total += weight;
            ++count;
          }
        }

        if (count >= min_valid_neighbors && weight_total > 0.0f) {
          result(buffer_row(row), buffer_col(col)) =
              weighted_sum / weight_total;
          mask(buffer_row(row), buffer_col(col)) = 1.0f;
        }
      }
    }
  }

  if (map.exists(layer::elevation_inpainted))
    map.get(layer::elevation_inpainted) = result;
  else
    map.add(layer::elevation_inpainted, result);

  if (map.exists(layer::inpainted_mask))
    map.get(layer::inpainted_mask) = mask;
  else
    map.add(layer::inpainted_mask, mask);

  if (inplace) map.get(layer::elevation) = result;
}

}  // namespace fastdem
