#include "sdquadx/consts.h"
#include "utils/eigen.h"

namespace sdquadx::wbc {

// [DIFF]
bool BuildContactConstraintMat(Eigen::Ref<Eigen::Matrix<fpt_t, 6, 3>> ret, fpt_t mu) {
  ret << 0, 0, 1, 1, 0, mu, -1, 0, mu, 0, 1, mu, 0, -1, mu, 0, 0, -1;
  return true;
}

// [DIFF] [0 0 0 0 0 -fmax]T
bool BuildContactConstraintUpperBoundVec(Eigen::Ref<Eigen::Matrix<fpt_t, 6, 1>> ret, fpt_t fmax) {
  ret.fill(0.);
  ret[5] = -fmax;
  return true;
}
}  // namespace sdquadx::wbc
