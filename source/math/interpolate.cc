#include "math/interpolate.h"

#include <cmath>

namespace sdquadx::math {

bool illegal_coeft(fpt_t t) {
  if (t < 1.e-12 || t - 1. > 1.e-12) return true;
  return false;
}

bool interpolate_linear(fpt_t &ret, fpt_t const y0, fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = y0 + (yf - y0) * t;
  return true;
}

bool interpolate_linear(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                        Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = y0 + (yf - y0) * t;
  return true;
}

bool interpolate_quadratic_bezier(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 2) * y0 + 2 * (1 - t) * t * y1 + std::pow(t, 2) * yf;
  return true;
}

bool interpolate_quadratic_bezier(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                  Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &yf,
                                  fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 2) * y0 + 2 * (1 - t) * t * y1 + std::pow(t, 2) * yf;
  return true;
}

bool interpolate_quadratic_bezier_derivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf,
                                             fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (1 - t) * (y1 - y0) + 2 * t * (yf - y1);
  return true;
}

bool interpolate_quadratic_bezier_derivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                             Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &yf,
                                             fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (1 - t) * (y1 - y0) + 2 * t * (yf - y1);
  return true;
}
bool interpolate_quadratic_bezier_second_derivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf,
                                                    fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (yf - 2 * y1 + y0);
  return true;
}

bool interpolate_quadratic_bezier_second_derivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                                    Eigen::Ref<VectorX const> const &y1,
                                                    Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 2 * (yf - 2 * y1 + y0);
  return true;
}

bool interpolate_cubic_bezier(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf,
                              fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 3) * y0 + 3 * std::pow((1 - t), 2) * t * y1 + 3 * (1 - t) * std::pow(t, 2) * y2 +
        std::pow(t, 3) * yf;
  return true;
}

bool interpolate_cubic_bezier(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                              Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                              Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = std::pow((1 - t), 3) * y0 + 3 * std::pow((1 - t), 2) * t * y1 + 3 * (1 - t) * std::pow(t, 2) * y2 +
        std::pow(t, 3) * yf;
  return true;
}

bool interpolate_cubic_bezier_derivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf,
                                         fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 3 * std::pow((1 - t), 2) * (y1 - y0) + 6 * (1 - t) * t * (y2 - y1) + 3 * std::pow(t, 2) * (yf - y2);
  return true;
}

bool interpolate_cubic_bezier_derivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                         Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                                         Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 3 * std::pow((1 - t), 2) * (y1 - y0) + 6 * (1 - t) * t * (y2 - y1) + 3 * std::pow(t, 2) * (yf - y2);
  return true;
}

bool interpolate_cubic_bezier_second_derivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2,
                                                fpt_t const yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 6 * (1 - t) * (y2 - 2 * y1 + y0) + 6 * t * (yf - 2 * y2 + y1);
  return true;
}

bool interpolate_cubic_bezier_second_derivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                                Eigen::Ref<VectorX const> const &y1,
                                                Eigen::Ref<VectorX const> const &y2,
                                                Eigen::Ref<VectorX const> const &yf, fpt_t const t) {
  if (illegal_coeft(t)) return false;
  ret = 6 * (1 - t) * (y2 - 2 * y1 + y0) + 6 * t * (yf - 2 * y2 + y1);
  return true;
}

}  // namespace sdquadx::math
