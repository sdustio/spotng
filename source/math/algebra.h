#pragma once

#include "eigen.h"

namespace sdrobot::math
{
  /*!
  * Compute the pseudo inverse of a matrix
  * @param ret : output matrix: pseudo inverse matrix of input matrix
  * @param inmat : input matrix
  * @param sigmaThreshold : threshold for singular values being zero
  */
  void PseudoInverse(Eigen::Ref<MatrixX> ret, Eigen::Ref<MatrixX const> const &inmat, double sigma_threshold);
}
