#pragma once

#include "sd/types.h"

namespace sd::dynamics
{
  /*!
  * Square a number
  */
  template <typename T>
  T Square(T a)
  {
    return a * a;
  }

  /*!
  * Compute the pseudo inverse of a matrix
  * @param matrix : input matrix
  * @param sigmaThreshold : threshold for singular values being zero
  * @param invMatrix : output matrix
  */
  void PseudoInverse(MatrixXd const &matrix, double sigmaThreshold,
                     MatrixXd &invMatrix);
}
