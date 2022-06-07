#pragma once

#include <array>
#include <cmath>

#include "spotng/consts.h"
#include "spotng/types.h"

namespace spotng::math {
/*!
 * Square a number
 */
template <typename T>
T Square(T a) {
  return a * a;
}

template <typename T>
bool HasNaN(T begin, T end) {
  while (begin != end) {
    if (std::isnan(*begin)) return true;
    begin++;
  }
  return false;
}

/*!
 * Convert radians to degrees 转化弧度到度数
 */
inline fpt_t RadToDeg(fpt_t rad) { return rad * 180.0 / consts::math::kPI; }

/*!
 * Convert degrees to radians
 */
inline fpt_t DegToRad(fpt_t deg) { return deg * consts::math::kPI / 180.0; }

inline fpt_t LimitV(fpt_t v, fpt_t max, fpt_t min) { return std::fmin(std::fmax(v, min), max); }

}  // namespace spotng::math
