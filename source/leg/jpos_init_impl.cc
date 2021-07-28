#include "leg/jpos_init_impl.h"
#include "math/interpolate.h"

namespace sdrobot::leg
{
  JPosInitImpl::JPosInitImpl(double dt, double time_end) : dt_(dt),
                                                           end_time_(time_end)
  {

    ini_jpos_ = {};
    target_jpos_ = {
        -0.6, -1.0, 2.7,
        0.6, -1.0, 2.7,
        -0.6, -1.0, 2.7,
        0.6, -1.0, 2.7};
    mid_jpos_ = {
        -1.8, 0., 2.7,
        1.8, 0., 2.7,
        -1.7, 0.5, 0.5,
        1.7, 0.5, 0.5};
  }

  bool JPosInitImpl::IsInitialized(LegCtrl::SharedPtr &ctrl)
  {
    curr_time_ += dt_;
    if (first_visit_)
    {
      UpdateInitial(ctrl);
      first_visit_ = false;
    }

    if (curr_time_ < end_time_)
    {
      Eigen::Matrix<fptype, params::model::num_act_joint, 1> jpos;
      Eigen::Map<Eigen::Matrix<fptype, params::model::num_act_joint, 1>> y0(ini_jpos_.data());
      Eigen::Map<Eigen::Matrix<fptype, params::model::num_act_joint, 1>> y1(mid_jpos_.data());
      Eigen::Map<Eigen::Matrix<fptype, params::model::num_act_joint, 1>> yf(target_jpos_.data());
      double t = curr_time_ / end_time_;

      math::interpolate_quadratic_bezier(jpos, y0, y1, yf, t);

      for (int leg(0); leg < params::model::num_leg; ++leg)
      {
        for (int jidx(0); jidx < params::model::num_leg_joint; ++jidx)
        {
          auto &cmds = ctrl->GetCmdsForUpdate();
          cmds[leg].tau_feed_forward[jidx] = 0.;
          cmds[leg].q_des[jidx] = jpos[3 * leg + jidx];
          cmds[leg].qd_des[jidx] = 0.;
        }
      }

      return false;
    }
    return true;
  }

  void JPosInitImpl::UpdateInitial(const LegCtrl::SharedPtr &ctrl)
  {
    for (int leg(0); leg < params::model::num_leg; ++leg)
    {
      for (int jidx(0); jidx < params::model::num_leg_joint; ++jidx)
      {
        int idx = 3 * leg + jidx;
        ini_jpos_[idx] = ctrl->GetDatas()[leg].q[jidx];
      }
    }

    SdMatrix3f kp, kd; // column major!!因为对称，所以 column major 和 row major 一致
    kp = {5, 0, 0,
          0, 5, 0,
          0, 0, 5};
    kd = {0.1, 0, 0,
          0, 0.1, 0,
          0, 0, 0.1};

    for (auto &cmd : ctrl->GetCmdsForUpdate())
    {
      cmd.kp_joint = kp;
      cmd.kd_joint = kd;
    }
  }

}
