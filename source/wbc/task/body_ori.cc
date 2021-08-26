#include "wbc/task/body_ori.h"

#include "dynamics/rotation.h"

namespace sdquadx::wbc {
using Jt_t = Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig>;

TaskBodyOri::TaskBodyOri(model::FloatBaseModel::ConstSharedPtr const &model, SdVector3f const &kp, SdVector3f const &kd)
    : Task(model, kp, kd) {
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 0).setIdentity();
  // kp_ori, kd_ori
}

bool TaskBodyOri::_UpdateCommand(SdVector3f const &pos_des, SdVector3f const &vel_des, SdVector3f const &acc_des) {
  Vector4 ori_cmd;
  dynamics::RPYToQuat(ori_cmd, ToConstEigenTp(pos_des));
  auto const &robot_ori = robot_sys_->GetState().ori;

  Vector4 link_ori_inv;
  link_ori_inv << robot_ori[0], -robot_ori[1], -robot_ori[2], -robot_ori[3];
  // link_ori_inv /= robot_ori.norm();

  // Explicit because operational space is in global frame
  Vector4 ori_err;
  dynamics::QuatProduct(ori_err, ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vector3 ori_err_so3;
  dynamics::QuatToSO3(ori_err_so3, ori_err);
  auto const &robot_vel = robot_sys_->GetState().gvel_robot;

  // Configuration space: Local
  // Operational Space: Global
  dynamics::RotMat rot;
  dynamics::QuatToRotMat(rot, ToConstEigenTp(robot_ori));
  Vector3 vel_err = rot.transpose() * (ToConstEigenTp(vel_des_) - ToConstEigenTp(robot_vel).head(3));

  for (int m = 0; m < 3; m++) {
    if (fabs(vel_err[m]) > 1000) {
      // std::cout << "body Ori Task vel_des_  " << vel_des_[m] << "\toverflow,
      // edit to  ";
      vel_err[m] = 0;
      // std::cout << vel_err[m] << std::endl;
    }
  }
  // Rx, Ry, Rz
  for (int i(0); i < 3; ++i) {
    pos_err_[i] = Kp_kin_[i] * ori_err_so3[i];
    vel_des_[i] = vel_des[i];
    acc_des_[i] = acc_des[i];

    op_cmd_[i] = Kp_[i] * ori_err_so3[i] + Kd_[i] * vel_err[i] + acc_des_[i];
    if (fabs(op_cmd_[i]) > 100) {
      // std::cout << "-------------------big
      // problem------------------------------" << std::endl; std::cout <<
      // "bodyOriTask acc error" << op_cmd_[i] << std::endl; std::cout <<
      // "ori_err_so3[i]" << ori_err_so3[i] << std::endl; std::cout <<
      // "vel_err[i]" << vel_err[i] << std::endl; std::cout << "acc_des_[i]" <<
      // acc_des_[i] << std::endl;
    }
  }

  return true;
}

bool TaskBodyOri::_UpdateTaskJacobian() {
  dynamics::RotMat rot;
  dynamics::QuatToRotMat(rot, ToConstEigenTp(robot_sys_->GetState().ori));
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 0) = rot.transpose();
  return true;
}

bool TaskBodyOri::_UpdateTaskJDotQdot() { return true; }

bool TaskBodyOri::_AdditionalUpdate() { return true; }
}  // namespace sdquadx::wbc
