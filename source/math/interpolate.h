#pragma once

#include "eigen.h"

namespace sdrobot::math
{

  /*!
 * Linear interpolation between y0 and yf.  t is between 0 and 1
 */
  bool interpolate_linear(fptype &ret, fptype const y0, fptype const yf, fptype const t);

  bool interpolate_linear(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

  /*!
 * Quadratic bezier interpolation between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
  bool interpolate_quadratic_bezier(fptype &ret, fptype const y0, fptype const y1, fptype const yf, fptype const t);

  bool interpolate_quadratic_bezier(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

  /*!
 * Quadratic bezier interpolation derivative between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
  bool interpolate_quadratic_bezier_derivative(fptype &ret, fptype const y0, fptype const y1, fptype const yf, fptype const t);

  bool interpolate_quadratic_bezier_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

  /*!
 * Quadratic bezier interpolation second derivative between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
  bool interpolate_quadratic_bezier_second_derivative(fptype &ret, fptype const y0, fptype const y1, fptype const yf, fptype const t);

  bool interpolate_quadratic_bezier_second_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

  /*!
 * Cubic bezier interpolation between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
  bool interpolate_cubic_bezier(fptype &ret, fptype const y0, fptype const y1, fptype const y2, fptype const yf, fptype const t);

  bool interpolate_cubic_bezier(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

  /*!
 * Cubic bezier interpolation derivative between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
  bool interpolate_cubic_bezier_derivative(fptype &ret, fptype const y0, fptype const y1, fptype const y2, fptype const yf, fptype const t);

  bool interpolate_cubic_bezier_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

  /*!
 * Cubic bezier interpolation second derivative between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
  bool interpolate_cubic_bezier_second_derivative(fptype &ret, fptype const y0, fptype const y1, fptype const y2, fptype const yf, fptype const t);

  bool interpolate_cubic_bezier_second_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      fptype const t);

}
