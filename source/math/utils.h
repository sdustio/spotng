#pragma once

#include <cmath>
#include "sdrobot/types.h"
#include "sdrobot/params.h"

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
  inline fpt_t RadToDeg(fpt_t rad)
  {
    return rad * 180.0 / params::math::kPI;
  }

  /*!
 * Convert degrees to radians
 */
  inline fpt_t DegToRad(fpt_t deg)
  {
    return deg * params::math::kPI / 180.0;
  }

}
