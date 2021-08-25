#pragma once

#include <cmath>

#include "sdrobot/consts.h"
#include "sdrobot/types.h"

namespace sdrobot::math {
/*!
 * Square a number
 */
template <typename T>
T Square(T a) {
  return a * a;
}

/*!
 * Convert radians to degrees 转化弧度到度数
 */
inline fpt_t RadToDeg(fpt_t rad) { return rad * 180.0 / consts::math::kPI; }

/*!
 * Convert degrees to radians
 */
inline fpt_t DegToRad(fpt_t deg) { return deg * consts::math::kPI / 180.0; }

}  // namespace sdrobot::math
