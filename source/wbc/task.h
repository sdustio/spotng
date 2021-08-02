#pragma once

#include "sdrobot/model.h"

namespace sdrobot::wbc
{

  class Task
  {
  public:
    using Jt_t = Eigen::Matrix<fpt_t, 3, params::model::dim_config>;

    Task(model::FloatBaseModel::SharedPtr const &model) : robot_sys_(model) {}

    bool UpdateTask(SdVector3f const &pos_des, SdVector3f const &vel_des,
                    SdVector3f const &acc_des)
    {
      _UpdateTaskJacobian();
      _UpdateTaskJDotQdot();
      _UpdateCommand(pos_des, vel_des, acc_des);
      _AdditionalUpdate();
      b_set_task_ = true;
      return true;
    }

    SdVector3f const &GetCommand() { return op_cmd_; }
    std::array<fpt_t, 3 * params::model::dim_config> const &GetTaskJacobian() { return Jt_; }
    SdVector3f const &GetTaskJacobianDotQdot() { return JtDotQdot_; }

    SdVector3f const &GetPosError() { return pos_err_; }
    SdVector3f const &GetDesVel() { return vel_des_; }
    SdVector3f const &GetDesAcc() { return acc_des_; }

  protected:
    // Update op_cmd_
    virtual bool _UpdateCommand(SdVector3f const &pos_des, SdVector3f const &vel_des,
                                SdVector3f const &acc_des) = 0;
    // Update Jt_
    virtual bool _UpdateTaskJacobian() = 0;
    // Update JtDotQdot_
    virtual bool _UpdateTaskJDotQdot() = 0;
    // Additional Update (defined in child classes)
    virtual bool _AdditionalUpdate() = 0;

    model::FloatBaseModel::SharedPtr const robot_sys_;

    bool b_set_task_ = false;

    SdVector3f op_cmd_;
    std::array<fpt_t, 3 * params::model::dim_config> Jt_;
    SdVector3f JtDotQdot_;

    SdVector3f pos_err_;
    SdVector3f vel_des_;
    SdVector3f acc_des_;
  };
} // namespace sdrobot::wbc
