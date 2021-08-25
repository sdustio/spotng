#pragma once

#include "wbc/task.h"

namespace sdrobot::wbc {
class TaskLinkPos : public Task {
 public:
  TaskLinkPos(model::FloatBaseModel::ConstSharedPtr const &model, SdVector3f const &kp, SdVector3f const &kd,
              int linkid, bool virtual_depend = true);

 private:
  // Update op_cmd_
  bool _UpdateCommand(SdVector3f const &pos_des, SdVector3f const &vel_des, SdVector3f const &acc_des);
  // Update Jt_
  bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  bool _UpdateTaskJDotQdot();
  // Additional Update (defined in child classes)
  bool _AdditionalUpdate();

  int link_idx_;
  bool virtual_depend_;
};

}  // namespace sdrobot::wbc
