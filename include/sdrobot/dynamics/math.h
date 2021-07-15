#pragma once

#include "sdrobot/types.h"

namespace sdrobot::dynamics
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
 * Convert radians to degrees 转化弧度到度数
 */
  double RadToDeg(double rad);

  /*!
 * Convert degrees to radians
 */
  double DegToRad(double deg);

  /*!
  * Compute the pseudo inverse of a matrix
  * @param matrix : input matrix
  * @param sigmaThreshold : threshold for singular values being zero
  * @param invMatrix : output matrix
  */
  void PseudoInverse(MatrixX const &matrix, double sigmaThreshold,
                     MatrixX &invMatrix);
}
