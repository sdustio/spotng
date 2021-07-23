#include "controllers/wbc/task/link_pos.h"

namespace sdrobot::ctrl::wbc
{
  TaskLinkPos::TaskLinkPos(
      const dynamics::FBModelPtr &model, int linkid, bool virtual_depend)
      : Task(3, model), link_idx_(linkid), virtual_depend_(virtual_depend)
  {
    Jt_ = MatrixX::Zero(dim_task_, robot::ModelAttrs::dim_config);
    JtDotQdot_ = VectorX::Zero(dim_task_);
    Kp_kin_ = VectorX::Constant(dim_task_, 1.);
    for (int i = 0; i < 3; i++)
    {
      Kp_[i] = robot::DynamicsAttrs::kp_foot[i];
      Kd_[i] = robot::DynamicsAttrs::kd_foot[i];
    }
  }

  bool TaskLinkPos::_UpdateCommand(const Vector3 &pos_des, const Vector3 &vel_des,
                                   const Vector3 &acc_des)
  {

    Vector3 link_pos;

    link_pos = robot_sys_->GetGcPos()[link_idx_];

    // X, Y, Z
    for (int i(0); i < 3; ++i)
    {
      pos_err_[i] = Kp_kin_[i] * (pos_des[i] - link_pos[i]);
      vel_des_[i] = vel_des[i];
      acc_des_[i] = acc_des[i];
    }

    // Op acceleration command
    for (int i(0); i < dim_task_; ++i)
    {
      op_cmd_[i] =
          Kp_[i] * pos_err_[i] +
          Kd_[i] * (vel_des_[i] - robot_sys_->GetGcVel()[link_idx_][i]) +
          acc_des_[i];
    }

    return true;
  }

  bool TaskLinkPos::_UpdateTaskJacobian()
  {

    Jt_ = robot_sys_->GetJc()[link_idx_];
    if (!virtual_depend_)
    {
      Jt_.block(0, 0, 3, 6) = MatrixX::Zero(3, 6);
    }
    return true;
  }

  bool TaskLinkPos::_UpdateTaskJDotQdot()
  {
    JtDotQdot_ = robot_sys_->GetJcdqd()[link_idx_];
    return true;
  }

  bool TaskLinkPos::_AdditionalUpdate() { return true; }

}
