#pragma once

#include "sdrobot/types.h"

namespace sdrobot::mpc {
class FootSwingTrajectory {
 public:
  FootSwingTrajectory();
  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  bool UpdateInitialPosition(SdVector3f const &p0) {
    p0_ = p0;
    return true;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  bool UpdateFinalPosition(SdVector3f const &pf) {
    pf_ = pf;
    return true;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the
   * swing
   */
  bool UpdateHeight(fpt_t h) {
    height_ = h;
    return true;
  }

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  SdVector3f const &GetPosition() { return p_; }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  SdVector3f const &GetVelocity() { return v_; }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  SdVector3f const &GetAcceleration() { return a_; }

  bool ComputeSwingTrajectoryBezier(fpt_t const phase, fpt_t const swingTime);

 private:
  SdVector3f p0_, pf_, p_, v_, a_;
  fpt_t height_ = 0;
};
}  // namespace sdrobot::mpc
