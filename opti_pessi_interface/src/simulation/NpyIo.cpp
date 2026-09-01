#include "opti_pessi_interface/simulation/NpyIo.h"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace opti_pessi {

namespace {

/** NPY v1.0 header: 6-byte magic, 2 version bytes, 2 length bytes, then a padded python dict. */
void writeNpyHeader(std::ofstream& f, const std::string& descr, const std::string& shape) {
  std::string header = "{'descr': '" + descr + "', 'fortran_order': False, 'shape': " + shape + ", }";
  constexpr size_t preamble = 10;  // magic (8) + header length (2)
  const size_t unpaddedLength = preamble + 2 + header.size() + 1;
  const size_t padding = (64 - (unpaddedLength % 64)) % 64;
  header.append(padding, ' ');
  header.push_back('\n');

  const auto headerLength = static_cast<uint16_t>(header.size());
  const unsigned char magic[] = {0x93, 'N', 'U', 'M', 'P', 'Y', 0x01, 0x00};
  f.write(reinterpret_cast<const char*>(magic), 8);
  f.write(reinterpret_cast<const char*>(&headerLength), 2);
  f.write(header.data(), static_cast<std::streamsize>(header.size()));
}

}  // namespace

void saveNpy(const std::string& path, const matrix_t& matrix) {
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("[NpyIo] cannot write " + path);
  }
  std::ostringstream shape;
  shape << "(" << matrix.rows() << ", " << matrix.cols() << ")";
  writeNpyHeader(f, "<f8", shape.str());
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      const double v = matrix(i, j);
      f.write(reinterpret_cast<const char*>(&v), sizeof(double));
    }
  }
}

void saveNpy3(const std::string& path, const std::vector<matrix_t>& slices) {
  if (slices.empty()) {
    saveNpy(path, matrix_t(0, 0));
    return;
  }
  const int numSlices = static_cast<int>(slices.size());
  const int rows = static_cast<int>(slices.front().rows());
  const int cols = static_cast<int>(slices.front().cols());

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("[NpyIo] cannot write " + path);
  }
  std::ostringstream shape;
  shape << "(" << numSlices << ", " << rows << ", " << cols << ")";
  writeNpyHeader(f, "<f8", shape.str());
  for (int i = 0; i < numSlices; ++i) {
    for (int j = 0; j < rows; ++j) {
      for (int k = 0; k < cols; ++k) {
        const double v = slices[static_cast<size_t>(i)](j, k);
        f.write(reinterpret_cast<const char*>(&v), sizeof(double));
      }
    }
  }
}

}  // namespace opti_pessi
