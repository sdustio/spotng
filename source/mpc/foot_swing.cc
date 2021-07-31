#include "sdrobot/dynamics/cubic_bezier.h"
#include "controllers/mpc/foot_swing.h"

namespace sdrobot::mpc
{
  FootSwingTrajectory::FootSwingTrajectory()
  {
    p0_.setZero();
    pf_.setZero();
    p_.setZero();
    v_.setZero();
    a_.setZero();
    height_ = 0;
  }

  void FootSwingTrajectory::ComputeSwingTrajectoryBezier(double phase, double swingTime)
  {
    p_ = dynamics::interpolate::cubicBezier<Vector3>(p0_, pf_, phase);
    v_ = dynamics::interpolate::cubicBezierFirstDerivative<Vector3>(p0_, pf_, phase) / swingTime;
    a_ = dynamics::interpolate::cubicBezierSecondDerivative<Vector3>(p0_, pf_, phase) / (swingTime * swingTime);

    double zp, zv, za;

    if (phase < 0.5)
    {
      zp = dynamics::interpolate::cubicBezier<double>(p0_[2], p0_[2] + height_, phase * 2);
      zv = dynamics::interpolate::cubicBezierFirstDerivative<double>(p0_[2], p0_[2] + height_, phase * 2) * 2 / swingTime;
      za = dynamics::interpolate::cubicBezierSecondDerivative<double>(p0_[2], p0_[2] + height_, phase * 2) * 4 / (swingTime * swingTime);
    }
    else
    {
      zp = dynamics::interpolate::cubicBezier<double>(p0_[2] + height_, pf_[2], phase * 2 - 1);
      zv = dynamics::interpolate::cubicBezierFirstDerivative<double>(p0_[2] + height_, pf_[2], phase * 2 - 1) * 2 / swingTime;
      za = dynamics::interpolate::cubicBezierSecondDerivative<double>(p0_[2] + height_, pf_[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    }

    p_[2] = zp;
    v_[2] = zv;
    a_[2] = za;
  }
}
