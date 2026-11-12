// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * feature_extraction.cpp
 *
 * PCA-based terrain feature extraction from elevation map.
 * Computes step, residual step, slope, roughness, curvature, and surface
 * normals.
 *
 *  Created on: Feb 2026
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/postprocess/feature_extraction.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <nanopcl/geometry/pca.hpp>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

void applyFeatureExtraction(ElevationMap& map, float analysis_radius,
                            int min_valid_neighbors,
                            float step_lower_percentile,
                            float step_upper_percentile,
                            int min_observations,
                            int max_observation_age_frames,
                            const char* ground_class_layer) {
  if (!map.exists(layer::elevation)) return;
  if (ground_class_layer != nullptr && !map.exists(ground_class_layer)) {
    throw std::invalid_argument("ground class layer does not exist");
  }

  // Ensure output layers exist
  if (!map.exists(layer::step)) map.add(layer::step, NAN);
  if (!map.exists(layer::step_residual)) map.add(layer::step_residual, NAN);
  if (!map.exists(layer::slope)) map.add(layer::slope, NAN);
  if (!map.exists(layer::roughness)) map.add(layer::roughness, NAN);
  if (!map.exists(layer::curvature)) map.add(layer::curvature, NAN);
  if (!map.exists(layer::normal_x)) map.add(layer::normal_x, NAN);
  if (!map.exists(layer::normal_y)) map.add(layer::normal_y, NAN);
  if (!map.exists(layer::normal_z)) map.add(layer::normal_z, NAN);

  const auto& elev = map.get(layer::elevation);
  auto& step_mat = map.get(layer::step);
  auto& step_residual_mat = map.get(layer::step_residual);
  auto& slope_mat = map.get(layer::slope);
  auto& roughness_mat = map.get(layer::roughness);
  auto& curvature_mat = map.get(layer::curvature);
  auto& nx_mat = map.get(layer::normal_x);
  auto& ny_mat = map.get(layer::normal_y);
  auto& nz_mat = map.get(layer::normal_z);

  // Feature layers may already exist when a caller reuses a map. Clear them
  // before recomputing so invalid or degenerate cells cannot retain old data.
  step_mat.setConstant(NAN);
  step_residual_mat.setConstant(NAN);
  slope_mat.setConstant(NAN);
  roughness_mat.setConstant(NAN);
  curvature_mat.setConstant(NAN);
  nx_mat.setConstant(NAN);
  ny_mat.setConstant(NAN);
  nz_mat.setConstant(NAN);

  const bool have_confidence = map.exists(layer::n_points) &&
                               map.exists(layer::observation_age);
  const auto* n_points = have_confidence ? &map.get(layer::n_points) : nullptr;
  const auto* observation_age =
      have_confidence ? &map.get(layer::observation_age) : nullptr;
  const auto* ground_class = ground_class_layer != nullptr ?
      &map.get(ground_class_layer) : nullptr;
  const auto supported = [&](const auto index) {
    if (!std::isfinite(elev(index))) return false;
    if (ground_class != nullptr) {
      const float cell_class = (*ground_class)(index);
      if (!std::isfinite(cell_class) || cell_class != 1.0f) return false;
    }
    if (!have_confidence) return true;
    const float observations = (*n_points)(index);
    const float age = (*observation_age)(index);
    return std::isfinite(observations) && observations >= min_observations &&
           std::isfinite(age) && age >= 0.0f && age <= max_observation_age_frames;
  };

  const auto reg = map.kernel(analysis_radius);
  const double res = map.getResolution();

  // Reusable buffer for percentile-based step computation
  std::vector<float> z_vals;
  z_vals.reserve(reg.entries.size());
  std::vector<float> residual_vals;
  residual_vals.reserve(reg.entries.size());

  for (auto cell : map.cells()) {
    const float center_z = elev(cell.index);
    if (!supported(cell.index)) continue;

    // Accumulate neighbors (single pass: covariance + z values)
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
    z_vals.clear();
    int count = 0;

    for (auto n : map.neighbors(cell, reg)) {
      const float nz = elev(n.index);
      if (!supported(n.index)) continue;

      // World-frame displacement (grid_map: row→-x, col→-y)
      const int dr = n.row - cell.row;
      const int dc = n.col - cell.col;
      const Eigen::Vector3f d(-dr * static_cast<float>(res),
                              -dc * static_cast<float>(res), nz - center_z);

      sum += d;
      sum_sq.noalias() += d * d.transpose();
      z_vals.push_back(nz);
      ++count;
    }

    if (count < min_valid_neighbors) continue;

    // Covariance matrix
    const float inv_n = 1.0f / static_cast<float>(count);
    const Eigen::Vector3f mean = sum * inv_n;
    const Eigen::Matrix3f cov = sum_sq * inv_n - mean * mean.transpose();

    // A scan line with height noise can have 3D rank two but no surface
    // support across the line. Require spread in both horizontal axes.
    const float xy_trace = cov(0, 0) + cov(1, 1);
    const float xy_discriminant = std::hypot(
        cov(0, 0) - cov(1, 1), 2.0f * cov(0, 1));
    const float xy_min_variance = 0.5f * (xy_trace - xy_discriminant);
    if (xy_min_variance <= static_cast<float>(res * res) * 0.01f) continue;

    // PCA — skip degenerate patches (collinear or insufficient spread)
    constexpr float kMinEigenvalue = 1e-8f;
    const auto pca = nanopcl::geometry::computePCA(cov);
    if (!pca.valid) continue;
    if (pca.eigenvalues(1) < kMinEigenvalue) continue;

    // Normal vector (smallest eigenvector, flipped upward)
    Eigen::Vector3f normal = pca.eigenvectors.col(0);
    if (normal.z() < 0.0f) normal = -normal;

    // Step: percentile-based range (robust to outliers)
    std::sort(z_vals.begin(), z_vals.end());
    const int lo = static_cast<int>(step_lower_percentile * (count - 1));
    const int hi = static_cast<int>(step_upper_percentile * (count - 1));
    step_mat(cell.index) = z_vals[hi] - z_vals[lo];

    // Residual step: remove the locally fitted plane first. A regular slope
    // therefore has a small residual even when its raw z range is large.
    residual_vals.clear();
    const float normal_z = std::abs(normal.z());
    if (normal_z > kMinEigenvalue) {
      for (auto n : map.neighbors(cell, reg)) {
        const float neighbor_z = elev(n.index);
        if (!supported(n.index)) continue;
        const int dr = n.row - cell.row;
        const int dc = n.col - cell.col;
        const Eigen::Vector3f d(-dr * static_cast<float>(res),
                                -dc * static_cast<float>(res),
                                neighbor_z - center_z);
        residual_vals.push_back(std::abs(normal.dot(d)) / normal_z);
      }
    }
    if (static_cast<int>(residual_vals.size()) >= min_valid_neighbors) {
      std::sort(residual_vals.begin(), residual_vals.end());
      const int residual_lo =
          static_cast<int>(step_lower_percentile * (residual_vals.size() - 1));
      const int residual_hi =
          static_cast<int>(step_upper_percentile * (residual_vals.size() - 1));
      step_residual_mat(cell.index) =
          residual_vals[residual_hi] - residual_vals[residual_lo];
    }

    slope_mat(cell.index) =
        std::acos(std::clamp(std::abs(normal.z()), 0.0f, 1.0f)) *
        180.0f / static_cast<float>(M_PI);
    roughness_mat(cell.index) = std::sqrt(std::max(pca.eigenvalues(0), 0.0f));
    const float trace = cov.trace();
    curvature_mat(cell.index) =
        (trace > 0.0f) ? std::abs(pca.eigenvalues(0) / trace) : 0.0f;
    nx_mat(cell.index) = normal.x();
    ny_mat(cell.index) = normal.y();
    nz_mat(cell.index) = normal.z();
  }
}

}  // namespace fastdem
