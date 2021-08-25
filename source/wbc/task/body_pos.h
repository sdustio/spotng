#pragma once

#include "wbc/task.h"

namespace sdrobot::wbc {
class TaskBodyPos : public Task {
 public:
  TaskBodyPos(model::FloatBaseModel::ConstSharedPtr const &model, SdVector3f const &kp, SdVector3f const &kd);

 private:
  // Update op_cmd_
  bool _UpdateCommand(SdVector3f const &pos_des, SdVector3f const &vel_des, SdVector3f const &acc_des);
  // Update Jt_
  bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  bool _UpdateTaskJDotQdot();
  // Additional Update (defined in child classes)
  bool _AdditionalUpdate();

  SdVector3f Kp_kin_, Kp_, Kd_;
};

}  // namespace sdrobot::wbc
