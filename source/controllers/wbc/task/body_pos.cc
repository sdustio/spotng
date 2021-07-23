#include "sdrobot/dynamics/rotation.h"

#include "controllers/wbc/task/body_pos.h"

namespace sdrobot::ctrl::wbc
{
  TaskBodyPos::TaskBodyPos(const dynamics::FBModelPtr &model) : Task(3, model)
  {
    Jt_ = MatrixX::Zero(dim_task_, robot::ModelAttrs::dim_config);
    Jt_.block<3, 3>(0, 3).setIdentity();
    JtDotQdot_ = VectorX::Zero(dim_task_);

    Kp_kin_ = VectorX::Constant(dim_task_, 1.);
    for (int i = 0; i < 3; i++)
    {
      Kp_[i] = robot::DynamicsAttrs::kp_body[i];
      Kd_[i] = robot::DynamicsAttrs::kd_body[i];
    }
  }

  // Update op_cmd_
  bool TaskBodyPos::_UpdateCommand(const Vector3 &pos_des, const Vector3 &vel_des,
                                   const Vector3 &acc_des)
  {
    const auto &link_pos = robot_sys_->GetState().body_position;
    auto rot = dynamics::QuatToRotMat(robot_sys_->GetState().body_orientation);
    auto curr_vel = robot_sys_->GetState().body_velocity;
    curr_vel.tail(3) = rot.transpose() * curr_vel.tail(3);

    // X, Y, Z
    for (int i(0); i < 3; ++i)
    {
      pos_err_[i] = Kp_kin_[i] * (pos_des[i] - link_pos[i]);
      vel_des_[i] = vel_des[i];
      acc_des_[i] = acc_des[i];

      op_cmd_[i] = Kp_[i] * (pos_des[i] - link_pos[i]) +
                   Kd_[i] * (vel_des_[i] - curr_vel[i + 3]) +
                   acc_des_[i];
    }

    return true;
  }

  bool TaskBodyPos::_UpdateTaskJacobian()
  {
    auto rot = dynamics::QuatToRotMat(robot_sys_->GetState().body_orientation);
    Jt_.block<3, 3>(0, 3) = rot.transpose();
    return true;
  }

  bool TaskBodyPos::_UpdateTaskJDotQdot()
  {
    return true;
  }

  bool TaskBodyPos::_AdditionalUpdate()
  {
    return true;
  }

}
