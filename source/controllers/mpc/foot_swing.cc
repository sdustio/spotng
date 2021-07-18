#include "sdrobot/dynamics/cubic_bezier.h"
#include "controllers/mpc/foot_swing.h"

namespace sdrobot::ctrl::mpc
{
  FootSwingTrajectory::FootSwingTrajectory()
  {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  void FootSwingTrajectory::computeSwingTrajectoryBezier(double phase, double swingTime)
  {
    _p = dynamics::interpolate::cubicBezier<Vector3>(_p0, _pf, phase);
    _v = dynamics::interpolate::cubicBezierFirstDerivative<Vector3>(_p0, _pf, phase) / swingTime;
    _a = dynamics::interpolate::cubicBezierSecondDerivative<Vector3>(_p0, _pf, phase) / (swingTime * swingTime);

    double zp, zv, za;

    if (phase < 0.5)
    {
      zp = dynamics::interpolate::cubicBezier<double>(_p0[2], _p0[2] + _height, phase * 2);
      zv = dynamics::interpolate::cubicBezierFirstDerivative<double>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
      za = dynamics::interpolate::cubicBezierSecondDerivative<double>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
    }
    else
    {
      zp = dynamics::interpolate::cubicBezier<double>(_p0[2] + _height, _pf[2], phase * 2 - 1);
      zv = dynamics::interpolate::cubicBezierFirstDerivative<double>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
      za = dynamics::interpolate::cubicBezierSecondDerivative<double>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    }

    _p[2] = zp;
    _v[2] = zv;
    _a[2] = za;
  }
}
