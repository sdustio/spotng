#pragma once

#include "sdrobot/types.h"

namespace sdrobot::ctrl::mpc
{
  class FootSwingTrajectory
  {
  public:
    FootSwingTrajectory();
    /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
    void SetInitialPosition(Vector3 p0)
    {
      p0_ = p0;
    }

    /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
    void SetFinalPosition(Vector3 pf)
    {
      pf_ = pf;
    }

    /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
    void SetHeight(double h)
    {
      height_ = h;
    }

    /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
    const Vector3 &GetPosition()
    {
      return p_;
    }

    /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
    const Vector3 &GetVelocity()
    {
      return v_;
    }

    /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
    const Vector3 &GetAcceleration()
    {
      return a_;
    }

    void ComputeSwingTrajectoryBezier(double phase, double swingTime);

  private:
    Vector3 p0_, pf_, p_, v_, a_;
    double height_;
  };
}
