#include "opti_pessi_interface/LipKinematics.h"

namespace opti_pessi {

vector_t lipMapScalar(const vector_t& x, const vector_t& u, scalar_t w, scalar_t mass, scalar_t inertia) {
  return lipMap(x, u, w, mass, inertia);
}

}  // namespace opti_pessi
