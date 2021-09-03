#include "leg/jpos_init_impl.h"

#include "math/interpolate.h"

namespace sdquadx::leg {

JPosInitImpl::JPosInitImpl(fpt_t dt, fpt_t time_end, SdVector3f const &kp, SdVector3f const &kd,
                           std::array<SdVector3f, 4> const &jpos)
    : dt_(dt), end_time_(time_end), kp_(kp), kd_(kd) {
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 3; j++) {
      target_jpos_[i * 3 + j] = jpos[i][j];
    }
  }
}

bool JPosInitImpl::IsInitialized(LegCtrl::SharedPtr const &ctrl) {
  curr_time_ += dt_;
  if (first_visit_) {
    UpdateInitial(ctrl);
    first_visit_ = false;
  }

  if (curr_time_ < end_time_) {
    Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> jpos;
    Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> const> y0(ini_jpos_.data());
    Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> const> y1(target_jpos_.data());
    Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> const> yf(target_jpos_.data());
    fpt_t t = curr_time_ / end_time_;

    math::interpolate_quadratic_bezier(jpos, y0, y1, yf, t);

    for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
      for (int jidx(0); jidx < consts::model::kNumLegJoint; ++jidx) {
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

bool JPosInitImpl::UpdateInitial(LegCtrl::SharedPtr const &ctrl) {
  for (int leg(0); leg < consts::model::kNumLeg; ++leg) {
    for (int jidx(0); jidx < consts::model::kNumLegJoint; ++jidx) {
      int idx = 3 * leg + jidx;
      ini_jpos_[idx] = ctrl->GetDatas()[leg].q[jidx];
    }
  }

  for (auto &cmd : ctrl->GetCmdsForUpdate()) {
    ToEigenTp(cmd.kp_joint).diagonal() = ToConstEigenTp(kp_);
    ToEigenTp(cmd.kd_joint).diagonal() = ToConstEigenTp(kd_);
  }
  return true;
}

}  // namespace sdquadx::leg
