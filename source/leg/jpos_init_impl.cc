#include "leg/jpos_init_impl.h"

#include "math/interpolate.h"

namespace sdrobot::leg {
namespace opts {

constexpr inline std::array<fpt_t, consts::model::kNumActJoint> const
    target_jpos = {-0.6, -1.0, 2.7, 0.6, -1.0, 2.7,
                   -0.6, -1.0, 2.7, 0.6, -1.0, 2.7};
constexpr inline std::array<fpt_t, consts::model::kNumActJoint> const mid_jpos =
    {-1.8, 0., 2.7, 1.8, 0., 2.7, -1.7, 0.5, 0.5, 1.7, 0.5, 0.5};

// 对角线矩阵，row major == column major
constexpr inline SdMatrix3f const kp_mat = {5, 0, 0, 0, 5, 0, 0, 0, 5};
constexpr inline SdMatrix3f const kd_mat = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
// kp_mat << 120, 0, 0, 0, 120, 0, 0, 0, 120;
// kd_mat << 4, 0, 0, 0, 4, 0, 0, 0, 4;
}  // namespace opts

JPosInitImpl::JPosInitImpl(fpt_t dt, fpt_t time_end)
    : dt_(dt), end_time_(time_end) {
}

bool JPosInitImpl::IsInitialized(LegCtrl::SharedPtr const &ctrl) {
  curr_time_ += dt_;
  if (first_visit_) {
    UpdateInitial(ctrl);
    first_visit_ = false;
  }

  if (curr_time_ < end_time_) {
    Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> jpos;
    Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> const> y0(
        ini_jpos_.data());
    Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> const> y1(
        opts::mid_jpos.data());
    Eigen::Map<Eigen::Matrix<fpt_t, consts::model::kNumActJoint, 1> const> yf(
        opts::target_jpos.data());
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
    cmd.kp_joint = opts::kp_mat;
    cmd.kd_joint = opts::kd_mat;
  }
  return true;
}

}  // namespace sdrobot::leg
