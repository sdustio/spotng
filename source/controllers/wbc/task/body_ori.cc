#include "sdrobot/dynamics/rotation.h"

#include "controllers/wbc/task/body_ori.h"

namespace sdrobot::ctrl::wbc
{
  TaskBodyOri::TaskBodyOri(const dynamics::FBModelPtr &model) : Task(3, model)
  {
    _Kp_kin = VectorXd::Constant(dim_task_, 1.);
    for (size_t i = 0; i < 3; i++)
    {
      _Kp[i] = robot::DynamicsAttrs::kp_ori[i];
      _Kd[i] = robot::DynamicsAttrs::kd_ori[i];
    }
  }

  bool TaskBodyOri::_UpdateCommand(const Vector3d &pos_des, const Vector3d &vel_des,
                                   const Vector3d &acc_des)
  {
    auto ori_cmd = dynamics::RPYToQuat(pos_des);
    const auto& link_ori = _robot_sys->GetState().body_orientation;
    dynamics::Quat link_ori_inv;

    link_ori_inv << link_ori[0], -link_ori[1], -link_ori[2], -link_ori[3];
    // link_ori_inv /= link_ori.norm();

    // Explicit because operational space is in global frame
    auto ori_err = dynamics::QuatProduct(ori_cmd, link_ori_inv);
    if (ori_err[0] < 0.)
    {
      ori_err *= (-1.);
    }
    auto ori_err_so3 = dynamics::QuatToSO3(ori_err);
    const auto& curr_vel = _robot_sys->GetState().body_velocity;

    // Configuration space: Local
    // Operational Space: Global
    auto rot = dynamics::QuatToRotMat(link_ori);
    Vector3d vel_err = rot.transpose() * (vel_des_ - curr_vel.head(3));

    for (int m = 0; m < 3; m++)
    {
      if (fabs(vel_err[m]) > 1000)
      {
        // std::cout << "body Ori Task vel_des_  " << vel_des_[m] << "\toverflow, edit to  ";
        vel_err[m] = 0;
        // std::cout << vel_err[m] << std::endl;
      }
    }
    // Rx, Ry, Rz
    for (int i(0); i < 3; ++i)
    {
      pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
      vel_des_[i] = vel_des[i];
      acc_des_[i] = acc_des[i];

      op_cmd_[i] = _Kp[i] * ori_err_so3[i] +
                       _Kd[i] * vel_err[i] + acc_des_[i];
      if (fabs(op_cmd_[i]) > 100)
      {
        // std::cout << "-------------------big problem------------------------------" << std::endl;
        // std::cout << "bodyOriTask acc error" << op_cmd_[i] << std::endl;
        // std::cout << "ori_err_so3[i]" << ori_err_so3[i] << std::endl;
        // std::cout << "vel_err[i]" << vel_err[i] << std::endl;
        // std::cout << "acc_des_[i]" << acc_des_[i] << std::endl;
      }
    }

    return true;
  }

  bool TaskBodyOri::_UpdateTaskJacobian()
  {
    auto rot = dynamics::QuatToRotMat(_robot_sys->GetState().body_orientation);
    Jt_.block(0, 0, 3, 3) = rot.transpose();
    return true;
  }

  bool TaskBodyOri::_UpdateTaskJDotQdot()
  {
    return true;
  }

  bool TaskBodyOri::_AdditionalUpdate()
  {
    return true;
  }
}
