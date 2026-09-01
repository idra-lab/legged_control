#pragma once

#include <string>
#include <vector>

#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/** Writes a 2-D Eigen matrix as a little-endian float64 C-order .npy file. */
void saveNpy(const std::string& path, const matrix_t& matrix);

/** Writes a stack of equally-sized matrices as a 3-D .npy of shape (slices, rows, cols). */
void saveNpy3(const std::string& path, const std::vector<matrix_t>& slices);

}  // namespace opti_pessi
