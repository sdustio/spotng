#pragma once

#include "utils/eigen.h"

namespace forax::math {

/*!
 * Linear interpolation between y0 and yf.  t is between 0 and 1
 */
bool InterpolateLinear(fpt_t &ret, fpt_t const y0, fpt_t const yf, fpt_t const t);

bool InterpolateLinear(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                       Eigen::Ref<VectorX const> const &yf, fpt_t const t);

/*!
 * Quadratic bezier interpolation between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
bool InterpolateQuadraticBezier(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf, fpt_t const t);

bool InterpolateQuadraticBezier(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &yf,
                                fpt_t const t);

/*!
 * Quadratic bezier interpolation derivative between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
bool InterpolateQuadraticBezierDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf, fpt_t const t);

bool InterpolateQuadraticBezierDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                          Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &yf,
                                          fpt_t const t);

/*!
 * Quadratic bezier interpolation second derivative between y0 and yf.
 * y1 is ctrl point
 * t is between 0 and 1
 */
bool InterpolateQuadraticBezierSecondDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const yf,
                                                fpt_t const t);

bool InterpolateQuadraticBezierSecondDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                                Eigen::Ref<VectorX const> const &y1,
                                                Eigen::Ref<VectorX const> const &yf, fpt_t const t);

/*!
 * Cubic bezier interpolation between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
bool InterpolateCubicBezier(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf, fpt_t const t);

bool InterpolateCubicBezier(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                            Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                            Eigen::Ref<VectorX const> const &yf, fpt_t const t);

/*!
 * Cubic bezier interpolation derivative between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
bool InterpolateCubicBezierDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf,
                                      fpt_t const t);

bool InterpolateCubicBezierDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                      Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                                      Eigen::Ref<VectorX const> const &yf, fpt_t const t);

/*!
 * Cubic bezier interpolation second derivative between y0 and yf.
 * y1, y2 are ctrl points
 * t is between 0 and 1
 */
bool InterpolateCubicBezierSecondDerivative(fpt_t &ret, fpt_t const y0, fpt_t const y1, fpt_t const y2, fpt_t const yf,
                                            fpt_t const t);

bool InterpolateCubicBezierSecondDerivative(Eigen::Ref<VectorX> ret, Eigen::Ref<VectorX const> const &y0,
                                            Eigen::Ref<VectorX const> const &y1, Eigen::Ref<VectorX const> const &y2,
                                            Eigen::Ref<VectorX const> const &yf, fpt_t const t);

}  // namespace forax::math
