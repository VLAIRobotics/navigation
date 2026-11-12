// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_POSTPROCESS_FEATURE_EXTRACTION_HPP
#define FASTDEM_POSTPROCESS_FEATURE_EXTRACTION_HPP

#include "fastdem/elevation_map.hpp"

namespace fastdem {

namespace layer {
constexpr auto step = "step";
constexpr auto step_residual = "step_residual";
constexpr auto slope = "slope";
constexpr auto roughness = "roughness";
constexpr auto curvature = "curvature";
constexpr auto normal_x = "_normal_x";
constexpr auto normal_y = "_normal_y";
constexpr auto normal_z = "_normal_z";
}  // namespace layer

/// @brief Extract terrain features from elevation map using local PCA.
///
/// For each measured cell, gathers 3D neighbor positions within
/// analysis_radius, computes PCA on the local surface patch, and
/// derives geometric features.
///
/// Input layer: elevation; when n_points and observation_age are present,
/// only supported, recent cells participate in the fit and residuals.
/// When ground_class_layer is provided, only cells whose class value is 1
/// participate as centers or neighbors; other classes are excluded.
/// Output layers: step, step_residual, slope, roughness, curvature, normal_x,
/// normal_y, normal_z
void applyFeatureExtraction(ElevationMap& map,
                            float analysis_radius = 0.3f,
                            int min_valid_neighbors = 4,
                            float step_lower_percentile = 0.05f,
                            float step_upper_percentile = 0.95f,
                            int min_observations = 3,
                            int max_observation_age_frames = 3,
                            const char* ground_class_layer = nullptr);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_FEATURE_EXTRACTION_HPP
