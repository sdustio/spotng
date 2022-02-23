#include "wbc/task/body_ori.h"

#include "dynamics/rotation.h"
#include "spdlog/spdlog.h"

namespace sdquadx::wbc {
using Jt_t = Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig>;

TaskBodyOri::TaskBodyOri(SdVector3f const &kp, SdVector3f const &kd) : Task(kp, kd) {
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 0).setIdentity();
}

bool TaskBodyOri::UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                             SdVector3f const &xdd_des) {
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 0) = ToConstEigenTp(estate.rot_mat);

  Vector4 ori_cmd;
  dynamics::RPYToQuat(ori_cmd, ToConstEigenTp(x_des));
  Vector4 link_ori_inv;
  link_ori_inv << estate.ori[0], -estate.ori[1], -estate.ori[2], -estate.ori[3];

  // Explicit because operational space is in global frame
  Vector4 ori_err;
  dynamics::QuatProduct(ori_err, ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  dynamics::QuatToSO3(ToEigenTp(x_err_), ori_err);

  xd_des_ = xd_des;

  // Configuration space: Local
  // Operational Space: Global
  Vector3 xd_err =
      ToConstEigenTp(estate.rot_mat) * (ToConstEigenTp(xd_des_) - ToConstEigenTp(estate.avel_robot));

  for (int m = 0; m < 3; m++) {
    if (fabs(xd_err[m]) > 1000) {
      spdlog::warn("body ori task xd_err reset to 0 from {}", xd_err[m]);
      xd_err[m] = 0;
    }
  }
  // Rx, Ry, Rz
  for (int i = 0; i < 3; ++i) {
    xdd_cmd_[i] = Kp_[i] * x_err_[i] + Kd_[i] * xd_err[i] + xdd_des[i];
    if (fabs(xdd_cmd_[i]) > 100) {
      spdlog::warn("xdd_cmd error: xdd_cmd_[{}]: {} ", i, xdd_cmd_[i]);
    }
  }

  return true;
}
}  // namespace sdquadx::wbc
