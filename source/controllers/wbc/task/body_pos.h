#pragma once

#include "sdrobot/controllers/wbc.h"

namespace sdrobot::ctrl::wbc
{
  class TaskBodyPos : public Task
  {
  public:
    TaskBodyPos(const dynamics::FBModelPtr &model);

  private:
    // Update op_cmd_
    bool _UpdateCommand(const Vector3d &pos_des, const Vector3d &vel_des,
                        const Vector3d &acc_des) { return true; }
    // Update Jt_
    bool _UpdateTaskJacobian() { return true; }
    // Update JtDotQdot_
    bool _UpdateTaskJDotQdot() { return true; }
    // Additional Update (defined in child classes)
    bool _AdditionalUpdate() { return true; }
  };

}
