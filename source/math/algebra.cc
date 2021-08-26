#include "math/algebra.h"

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

namespace sdquadx::math {

/*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix 向量转反对称阵
 */
bool VecToSkewMat(Eigen::Ref<Matrix3> ret, Eigen::Ref<Vector3 const> const &v) {
  ret << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return true;
}

/*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector
 * 反对称阵转向量
 */
bool MatToSkewVec(Eigen::Ref<Vector3> ret, Eigen::Ref<Matrix3 const> const &m) {
  ret = 0.5 * Vector3(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
  return true;
}

bool PseudoInverse(Eigen::Ref<MatrixX> ret, Eigen::Ref<MatrixX const> const &inmat, fpt_t sigma_threshold) {
  if (!(ret.rows() == inmat.cols() && ret.cols() == inmat.rows())) {
    return false;
  }

  if ((1 == inmat.rows()) && (1 == inmat.cols())) {
    if (inmat.coeff(0, 0) > sigma_threshold) {
      ret.coeffRef(0, 0) = 1.0 / inmat.coeff(0, 0);
    } else {
      ret.coeffRef(0, 0) = 0.0;
    }
    return true;
  }

  Eigen::JacobiSVD<MatrixX> svd(inmat, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  MatrixX invS;
  invS = MatrixX::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigma_threshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      // invS.coeffRef(ii, ii) = 1.0/ sigma_threshold;
      // printf("sigular value is too small: %f\n",
      // svd.singularValues().coeff(ii));
    }
  }
  ret = svd.matrixV() * invS * svd.matrixU().transpose();
  return true;
}
}  // namespace sdquadx::math
