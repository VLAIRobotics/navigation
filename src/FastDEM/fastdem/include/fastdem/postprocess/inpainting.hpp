// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * inpainting.hpp
 *
 * Iterative neighbor averaging for filling NaN holes in height maps.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_POSTPROCESS_INPAINTING_HPP
#define FASTDEM_POSTPROCESS_INPAINTING_HPP

#include "fastdem/elevation_map.hpp"

namespace fastdem {

namespace layer {
constexpr auto elevation_inpainted = "elevation_inpainted";
constexpr auto inpainted_mask = "inpainted_mask";
}  // namespace layer

/**
 * @brief Fills bounded NaN regions using only original elevation samples.
 *
 * @param map Height map to process
 * @param max_iterations Maximum support radius in cells (default: 3)
 * @param min_valid_neighbors Minimum original samples required per cell
 * @param inplace If true, writes to elevation layer directly.
 *                If false, writes to elevation_inpainted layer (original unchanged).
 * @param max_hole_cells Maximum enclosed NaN component size eligible for filling
 */
void applyInpainting(ElevationMap& map, int max_iterations = 3,
                     int min_valid_neighbors = 4, bool inplace = false);

void applyInpainting(ElevationMap& map, int max_iterations,
                     int min_valid_neighbors, bool inplace,
                     int max_hole_cells);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_INPAINTING_HPP
