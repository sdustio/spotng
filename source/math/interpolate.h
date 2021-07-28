#pragma once

#include "eigen.h"

namespace sdrobot::math
{

  /*!
 * Linear interpolation between y0 and yf.  t is between 0 and 1
 */
  bool interpolate_linear(double &ret, double const y0, double const yf, double const t);

  bool interpolate_linear(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

  /*!
 * Quadratic bezier interpolation between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
  bool interpolate_quadratic_bezier(double &ret, double const y0, double const y1, double const yf, double const t);

  bool interpolate_quadratic_bezier(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

  /*!
 * Quadratic bezier interpolation derivative between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
  bool interpolate_quadratic_bezier_derivative(double &ret, double const y0, double const y1, double const yf, double const t);

  bool interpolate_quadratic_bezier_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

  /*!
 * Quadratic bezier interpolation second derivative between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
  bool interpolate_quadratic_bezier_second_derivative(double &ret, double const y0, double const y1, double const yf, double const t);

  bool interpolate_quadratic_bezier_second_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

  /*!
 * Cubic bezier interpolation between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
  bool interpolate_cubic_bezier(double &ret, double const y0, double const y1, double const y2, double const yf, double const t);

  bool interpolate_cubic_bezier(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

  /*!
 * Cubic bezier interpolation derivative between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
  bool interpolate_cubic_bezier_derivative(double &ret, double const y0, double const y1, double const y2, double const yf, double const t);

  bool interpolate_cubic_bezier_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

  /*!
 * Cubic bezier interpolation second derivative between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
  bool interpolate_cubic_bezier_second_derivative(double &ret, double const y0, double const y1, double const y2, double const yf, double const t);

  bool interpolate_cubic_bezier_second_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      double const t);

}
