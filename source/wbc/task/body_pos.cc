#include "wbc/task/body_pos.h"

#include "dynamics/rotation.h"

namespace sdquadx::wbc {
using Jt_t = Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig>;

TaskBodyPos::TaskBodyPos(model::FloatBaseModel::ConstSharedPtr const &model, SdVector3f const &kp, SdVector3f const &kd)
    : Task(model, kp, kd) {
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 0).setIdentity();
  // kp_body, kd_body
}

// Update op_cmd_
bool TaskBodyPos::_UpdateCommand(SdVector3f const &pos_des, SdVector3f const &vel_des, SdVector3f const &acc_des) {
  auto const &robot_pos = robot_sys_->GetState().pos;
  dynamics::RotMat rot;
  dynamics::QuatToRotMat(rot, ToConstEigenTp(robot_sys_->GetState().ori));
  Vector6 curr_vel = ToConstEigenTp(robot_sys_->GetState().gvel_robot);
  curr_vel.tail(3) = rot.transpose() * curr_vel.tail(3);

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    pos_err_[i] = Kp_kin_[i] * (pos_des[i] - robot_pos[i]);
    vel_des_[i] = vel_des[i];
    acc_des_[i] = acc_des[i];

    op_cmd_[i] = Kp_[i] * (pos_des[i] - robot_pos[i]) + Kd_[i] * (vel_des_[i] - curr_vel[i + 3]) + acc_des_[i];
  }

  return true;
}

bool TaskBodyPos::_UpdateTaskJacobian() {
  dynamics::RotMat rot;
  dynamics::QuatToRotMat(rot, ToConstEigenTp(robot_sys_->GetState().ori));
  Eigen::Map<Jt_t> Jt(Jt_.data());
  Jt.block<3, 3>(0, 0) = rot.transpose();
  return true;
}

bool TaskBodyPos::_UpdateTaskJDotQdot() { return true; }

bool TaskBodyPos::_AdditionalUpdate() { return true; }

}  // namespace sdquadx::wbc
