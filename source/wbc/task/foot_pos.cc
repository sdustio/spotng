#include "wbc/task/foot_pos.h"

#include "utils/eigen.h"

namespace forax::wbc {
using Jt_t = Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig>;

TaskFootPos::TaskFootPos(SdVector3f const &kp, SdVector3f const &kd, model::Quadruped::ConstSharedPtr const &quad,
                         int leg)
    : Task(kp, kd), mquad_(quad), leg_(leg) {}

bool TaskFootPos::UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                             SdVector3f const &xdd_des) {
  auto const &dyndata = mquad_->GetDynamicsData();
  Jt_ = dyndata.Jc[leg_];
  Jtdqd_ = dyndata.Jcdqd[leg_];

  for (int i = 0; i < 3; i++) {
    x_err_[i] = x_des[i] - estate.foot_pos[leg_][i];
    xd_des_[i] = xd_des[i];
    xdd_cmd_[i] = Kp_[i] * x_err_[i] + Kd_[i] * (xd_des_[i] - estate.foot_vel[leg_][i]) + xdd_des[i];
  }

  return true;
}
}  // namespace forax::wbc
