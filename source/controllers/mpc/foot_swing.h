#pragma once

#include "sdrobot/types.h"

namespace sdrobot::ctrl::mpc
{
  class FootSwingTrajectory
  {
  public:
    FootSwingTrajectory(){}
    /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
    void setInitialPosition(Vector3 p0)
    {
      _p0 = p0;
    }

    /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
    void setFinalPosition(Vector3 pf)
    {
      _pf = pf;
    }

    /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
    void setHeight(double h)
    {
      _height = h;
    }

    /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
    const Vector3 &getPosition()
    {
      return _p;
    }

    /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
    const Vector3 &getVelocity()
    {
      return _v;
    }

    /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
    const Vector3 &getAcceleration()
    {
      return _a;
    }

    void computeSwingTrajectoryBezier(double phase, double swingTime){}

  private:
    Vector3 _p0, _pf, _p, _v, _a;
    double _height;
  };
}
