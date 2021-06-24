#include "sd/controllers/jpos_init.h"
#include "sd/robot/runner.h"

namespace sd::ctrl
{
  using robot::ctrlparams::kCtrlsec;

  JPosInit::JPosInit() : end_time_(kEndTime), ini_jpos_{},
                         target_jpos_{
                             -0.6, -1.0, 2.7,
                             0.6, -1.0, 2.7,
                             -0.6, -1.0, 2.7,
                             0.6, -1.0, 2.7},
                         mid_jpos_{
                             -1.8, 0., 2.7,
                             1.8, 0., 2.7,
                             -1.7, 0.5, 0.5,
                             1.7, 0.5, 0.5}
  {
  }

  bool JPosInit::IsInitialized(LegPtr &ctrl)
  {
    curr_time_ += kCtrlsec;
    if (first_visit_)
    {
      UpdateInitial(ctrl);
      first_visit_ = false;
    }

    if (curr_time_ < end_time_)
    {
      std::array<double, dynamics::bspline::kDim> jpos;
      jpos_trj_.GetCurvePoint(curr_time_, jpos);

      for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
      {
        for (int jidx(0); jidx < robot::ModelAttrs::num_leg_joint; ++jidx)
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

  void JPosInit::UpdateInitial(const LegPtr &ctrl)
  {
    std::array<double, 3 * dynamics::bspline::kDim> ini{};
    std::array<double, 3 * dynamics::bspline::kDim> fin{};
    std::array<std::array<double, dynamics::bspline::kDim>, dynamics::bspline::kNumMiddle> mid{mid_jpos_};

    for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      for (int jidx(0); jidx < robot::ModelAttrs::num_leg_joint; ++jidx)
      {
        int idx = 3 * leg + jidx;
        ini_jpos_[idx] = ini[idx] = ctrl->GetDatas()[leg].q[jidx];
      }
    }

    for (int i(0); i < dynamics::bspline::kDim; ++i)
    {
      fin[i] = target_jpos_[i];
    }

    jpos_trj_.SetParam(ini, fin, mid, end_time_);

    Matrix3d kp, kd;
    kp << 5, 0, 0, 0, 5, 0, 0, 0, 5;
    kd << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;

    for (auto &cmd : ctrl->GetCmdsForUpdate())
    {
      cmd.kp_joint = kp;
      cmd.kd_joint = kd;
    }
  }

}
