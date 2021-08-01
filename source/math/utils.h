#pragma once

#include <cmath>

namespace sdrobot::math
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
  fpt_t RadToDeg(fpt_t rad)
  {
    return rad * 180.0 / M_PI;
  }

  /*!
 * Convert degrees to radians
 */
  fpt_t DegToRad(fpt_t deg)
  {
    return deg * M_PI / 180.0;
  }

}
