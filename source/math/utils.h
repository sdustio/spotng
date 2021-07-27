#pragma once

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
  double RadToDeg(double rad)
  {
    return rad * 180.0 / M_PI;
  }

  /*!
 * Convert degrees to radians
 */
  double DegToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

}
