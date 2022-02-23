#include "wbc/task/body_pos.h"

#include "dynamics/rotation.h"

namespace sdquadx::wbc {
using Jt_t = Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig>;

TaskBodyPos::TaskBodyPos(SdVector3f const &kp, SdVector3f const &kd) : Task(kp, kd) {
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 3).setIdentity();
}

bool TaskBodyPos::UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                             SdVector3f const &xdd_des) {
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 3) = ToConstEigenTp(estate.rot_mat);

  for (int i = 0; i < 3; i++) {
    x_err_[i] = x_des[i] - estate.pos[i];
    xd_des_[i] = xd_des[i];
    xdd_cmd_[i] = Kp_[i] * x_err_[i] + Kd_[i] * (xd_des_[i] - estate.lvel[i]) + xdd_des[i];
  }

  return true;
}

}  // namespace sdquadx::wbc
