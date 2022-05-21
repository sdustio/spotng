#include "math/interpolate.h"

#include <cmath>

#include "forax/consts.h"

namespace forax::math {

bool illegal_coeft(fpt_t t) { return t < consts::math::kZeroEpsilon || t > 1. - consts::math::kZeroEpsilon; }

bool InterpolateLinear(fpt_t &ret, fpt_t const y0, fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = y0 + (yf - y0) * t;
  return true;
}

bool InterpolateLinear(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                       Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = y0 + (yf - y0) * t;
  return true;
}

bool InterpolateQuadraticBezier(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 2) * y0 + 2 * (1 - t) * t * y1 + std::pow(t, 2) * yf;
  return true;
}

bool InterpolateQuadraticBezier(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &yf,
                                fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 2) * y0 + 2 * (1 - t) * t * y1 + std::pow(t, 2) * yf;
  return true;
}

bool InterpolateQuadraticBezierDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (1 - t) * (y1 - y0) + 2 * t * (yf - y1);
  return true;
}

bool InterpolateQuadraticBezierDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                          Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &yf,
                                          fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (1 - t) * (y1 - y0) + 2 * t * (yf - y1);
  return true;
}
bool InterpolateQuadraticBezierSecondDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf,
                                                fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (yf - 2 * y1 + y0);
  return true;
}

bool InterpolateQuadraticBezierSecondDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                                Eigen::Ref<VectorX const> const &y1,
                                                Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (yf - 2 * y1 + y0);
  return true;
}

bool InterpolateCubicBezier(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 3) * y0 + 3 * std::pow((1 - t), 2) * t * y1 + 3 * (1 - t) * std::pow(t, 2) * y2 +
        std::pow(t, 3) * yf;
  return true;
}

bool InterpolateCubicBezier(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                            Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                            Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 3) * y0 + 3 * std::pow((1 - t), 2) * t * y1 + 3 * (1 - t) * std::pow(t, 2) * y2 +
        std::pow(t, 3) * yf;
  return true;
}

bool InterpolateCubicBezierDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf,
                                      fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 3 * std::pow((1 - t), 2) * (y1 - y0) + 6 * (1 - t) * t * (y2 - y1) + 3 * std::pow(t, 2) * (yf - y2);
  return true;
}

bool InterpolateCubicBezierDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                      Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                                      Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 3 * std::pow((1 - t), 2) * (y1 - y0) + 6 * (1 - t) * t * (y2 - y1) + 3 * std::pow(t, 2) * (yf - y2);
  return true;
}

bool InterpolateCubicBezierSecondDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf,
                                            fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 6 * (1 - t) * (y2 - 2 * y1 + y0) + 6 * t * (yf - 2 * y2 + y1);
  return true;
}

bool InterpolateCubicBezierSecondDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                            Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                                            Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 6 * (1 - t) * (y2 - 2 * y1 + y0) + 6 * t * (yf - 2 * y2 + y1);
  return true;
}

}  // namespace forax::math
