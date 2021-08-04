#pragma once

#include "eigen.h"

namespace sdrobot::math
{

  /*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix 向量转反对称阵
 */
  bool VecToSkewMat(Eigen::Ref<Matrix3> ret, Eigen::Ref<Vector3 const> const &v);

  /*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector 反对称阵转向量
 */
  bool MatToSkewVec(Eigen::Ref<Vector3> ret, Eigen::Ref<Matrix3 const> const &m);

  /*!
  * Compute the pseudo inverse of a matrix
  * @param ret : output matrix: pseudo inverse matrix of input matrix
  * @param inmat : input matrix
  * @param sigmaThreshold : threshold for singular values being zero
  */
  bool PseudoInverse(Eigen::Ref<MatrixX> ret, Eigen::Ref<MatrixX const> const &inmat, fpt_t sigma_threshold);
}
