#include "mpc/foot_swing.h"

#include "math/interpolate.h"

namespace sdquadx::mpc {
FootSwingTrajectory::FootSwingTrajectory() {
  p0_.fill(0.);
  pf_.fill(0.);
  p_.fill(0.);
  v_.fill(0.);
  a_.fill(0.);
}

bool FootSwingTrajectory::ComputeSwingTrajectoryBezier(fpt_t const phase, fpt_t const swingTime) {
  auto p = ToEigenTp(p_);
  auto v = ToEigenTp(v_);
  auto a = ToEigenTp(a_);
  auto p0 = ToConstEigenTp(p0_);
  auto pf = ToConstEigenTp(pf_);

  math::interpolate_cubic_bezier(p, p0, p0, pf, pf, phase);
  math::interpolate_cubic_bezier_derivative(v, p0, p0, pf, pf, phase);
  v = v / swingTime;
  math::interpolate_cubic_bezier_second_derivative(a, p0, p0, pf, pf, phase);
  a = a / (swingTime * swingTime);

  fpt_t zp, zv, za;

  if (phase < 0.5) {
    math::interpolate_cubic_bezier(zp, p0_[2], p0_[2], p0_[2] + height_, p0_[2] + height_, phase * 2);
    math::interpolate_cubic_bezier_derivative(zv, p0_[2], p0_[2], p0_[2] + height_, p0_[2] + height_, phase * 2);
    zv = zv * 2 / swingTime;
    math::interpolate_cubic_bezier_second_derivative(za, p0_[2], p0_[2], p0_[2] + height_, p0_[2] + height_, phase * 2);
    za = za * 4 / (swingTime * swingTime);
  } else {
    math::interpolate_cubic_bezier(zp, p0_[2] + height_, p0_[2] + height_, pf_[2], pf_[2], phase * 2 - 1);
    math::interpolate_cubic_bezier_derivative(zv, p0_[2] + height_, p0_[2] + height_, pf_[2], pf_[2], phase * 2 - 1);
    zv = zv * 2 / swingTime;
    math::interpolate_cubic_bezier_second_derivative(za, p0_[2] + height_, p0_[2] + height_, pf_[2], pf_[2],
                                                     phase * 2 - 1);
    za = za * 4 / (swingTime * swingTime);
  }

  p_[2] = zp;
  v_[2] = zv;
  a_[2] = za;

  return true;
}
}  // namespace sdquadx::mpc
