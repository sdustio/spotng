#include "controllers/wbc/task/link_pos.h"

namespace sdrobot::ctrl::wbc
{
  TaskLinkPos::TaskLinkPos(
      const dynamics::FBModelPtr &model, size_t linkid, bool virtual_depend)
      : Task(3, model), link_idx_(linkid), virtual_depend_(virtual_depend)
  {
    Jt_ = MatrixXd::Zero(dim_task_, robot::ModelAttrs::dim_config);
    JtDotQdot_ = VectorXd::Zero(dim_task_);
    _Kp_kin = VectorXd::Constant(dim_task_, 1.);
    for (size_t i = 0; i < 3; i++)
    {
      _Kp[i] = robot::DynamicsAttrs::kp_foot[i];
      _Kd[i] = robot::DynamicsAttrs::kd_foot[i];
    }
  }

  bool TaskLinkPos::_UpdateCommand(const Vector3d &pos_des, const Vector3d &vel_des,
                                   const Vector3d &acc_des)
  {

    Vector3d link_pos;

    link_pos = _robot_sys->GetGcPos()[link_idx_];

    // X, Y, Z
    for (int i(0); i < 3; ++i)
    {
      pos_err_[i] = _Kp_kin[i] * (pos_des[i] - link_pos[i]);
      vel_des_[i] = vel_des[i];
      acc_des_[i] = acc_des[i];
    }

    // Op acceleration command
    for (size_t i(0); i < dim_task_; ++i)
    {
      op_cmd_[i] =
          _Kp[i] * pos_err_[i] +
          _Kd[i] * (vel_des_[i] - _robot_sys->GetGcVel()[link_idx_][i]) +
          acc_des_[i];
    }

    return true;
  }

  bool TaskLinkPos::_UpdateTaskJacobian()
  {

    Jt_ = _robot_sys->GetJc()[link_idx_];
    if (!virtual_depend_)
    {
      Jt_.block(0, 0, 3, 6) = MatrixXd::Zero(3, 6);
    }
    return true;
  }

  bool TaskLinkPos::_UpdateTaskJDotQdot()
  {
    JtDotQdot_ = _robot_sys->GetJcdqd()[link_idx_];
    return true;
  }

  bool TaskLinkPos::_AdditionalUpdate() { return true; }

}
