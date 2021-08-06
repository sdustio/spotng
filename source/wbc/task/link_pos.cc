#include "wbc/task/link_pos.h"
#include "eigen.h"

namespace sdrobot::wbc
{
  using Jt_t = Eigen::Matrix<fpt_t, 3, params::model::kDimConfig>;

  TaskLinkPos::TaskLinkPos(
      model::FloatBaseModel::ConstSharedPtr const &model,
      SdVector3f const &kp, SdVector3f const &kd,
      int linkid, bool virtual_depend)
      : Task(model, kp, kd), link_idx_(linkid), virtual_depend_(virtual_depend)
  {
    //kp_foot, kd_foot
  }

  bool TaskLinkPos::_UpdateCommand(SdVector3f const &pos_des, SdVector3f const &vel_des,
                                   SdVector3f const &acc_des)
  {

    auto const &link_pos = robot_sys_->GetGroundContactPos()[link_idx_];

    // X, Y, Z
    for (int i(0); i < 3; ++i)
    {
      pos_err_[i] = Kp_kin_[i] * (pos_des[i] - link_pos[i]);
      vel_des_[i] = vel_des[i];
      acc_des_[i] = acc_des[i];
    }

    // Op acceleration command
    for (int i(0); i < 3; ++i)
    {
      op_cmd_[i] =
          Kp_[i] * pos_err_[i] +
          Kd_[i] * (vel_des_[i] - robot_sys_->GetGroundContactVel()[link_idx_][i]) +
          acc_des_[i];
    }

    return true;
  }

  bool TaskLinkPos::_UpdateTaskJacobian()
  {

    Jt_ = robot_sys_->GetContactJacobians()[link_idx_];
    if (!virtual_depend_)
    {
      Eigen::Map<Jt_t> Jt(Jt_.data());
      Jt.block<3, 6>(0, 0) = MatrixX::Zero(3, 6);
    }
    return true;
  }

  bool TaskLinkPos::_UpdateTaskJDotQdot()
  {
    JtDotQdot_ = robot_sys_->GetContactJacobiansdqd()[link_idx_];
    return true;
  }

  bool TaskLinkPos::_AdditionalUpdate() { return true; }

}
