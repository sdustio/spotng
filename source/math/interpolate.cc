#include <cmath>
#include "math/interpolate.h"

namespace sdrobot::math
{
  bool interpolate_linear(double &ret, double const y0, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = y0 + (yf - y0) * t;
    return true;
  }

  bool interpolate_linear(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = y0 + (yf - y0) * t;
    return true;
  }

  bool interpolate_quadratic_bezier(double &ret, double const y0, double const y1, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = std::pow((1 - t), 2) * y0 + 2 * (1 - t) * t * y1 + std::pow(t, 2) * yf;
    return true;
  }

  bool interpolate_quadratic_bezier(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = std::pow((1 - t), 2) * y0 + 2 * (1 - t) * t * y1 + std::pow(t, 2) * yf;
    return true;
  }

  bool interpolate_quadratic_bezier_derivative(double &ret, double const y0, double const y1, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 2 * (1 - t) * (y1 - y0) + 2 * t * (yf - y1);
    return true;
  }

  bool interpolate_quadratic_bezier_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 2 * (1 - t) * (y1 - y0) + 2 * t * (yf - y1);
    return true;
  }
  bool interpolate_quadratic_bezier_second_derivative(double &ret, double const y0, double const y1, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 2 * (yf - 2 * y1 + y0);
    return true;
  }

  bool interpolate_quadratic_bezier_second_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 2 * (yf - 2 * y1 + y0);
    return true;
  }

  bool interpolate_cubic_bezier(double &ret, double const y0, double const y1, double const y2, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = std::pow((1 - t), 3) * y0 + 3 * std::pow((1 - t), 2) * t * y1 + 3 * (1 - t) * std::pow(t, 2) * y2 + std::pow(t, 3) * yf;
    return true;
  }

  bool interpolate_cubic_bezier(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = std::pow((1 - t), 3) * y0 + 3 * std::pow((1 - t), 2) * t * y1 + 3 * (1 - t) * std::pow(t, 2) * y2 + std::pow(t, 3) * yf;
    return true;
  }

  bool interpolate_cubic_bezier_derivative(double &ret, double const y0, double const y1, double const y2, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 3 * std::pow((1 - t), 2) * (y1 - y0) + 6 * (1 - t) * t * (y2 - y1) + 3 * std::pow(t, 2) * (yf - y2);
    return true;
  }

  bool interpolate_cubic_bezier_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 3 * std::pow((1 - t), 2) * (y1 - y0) + 6 * (1 - t) * t * (y2 - y1) + 3 * std::pow(t, 2) * (yf - y2);
    return true;
  }

  bool interpolate_cubic_bezier_second_derivative(double &ret, double const y0, double const y1, double const y2, double const yf, double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 6 * (1 - t) * (y2 - 2 * y1 + y0) + 6 * t * (yf - 2 * y2 + y1);
    return true;
  }

  bool interpolate_cubic_bezier_second_derivative(
      Eigen::Ref<VectorX> ret,
      Eigen::Ref<VectorX const> const &y0,
      Eigen::Ref<VectorX const> const &y1,
      Eigen::Ref<VectorX const> const &y2,
      Eigen::Ref<VectorX const> const &yf,
      double const t)
  {
    if (t < 0. || t > 1.)
      return false;
    ret = 6 * (1 - t) * (y2 - 2 * y1 + y0) + 6 * t * (yf - 2 * y2 + y1);
    return true;
  }

}
