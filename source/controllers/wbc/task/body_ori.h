#pragma once

#include "sdrobot/controllers/wbc.h"

namespace sdrobot::ctrl::wbc
{
  class TaskBodyOri : public Task
  {
  public:
    TaskBodyOri(const dynamics::FBModelPtr &model);

  private:
    // Update op_cmd_
    bool _UpdateCommand(const Vector3 &pos_des, const Vector3 &vel_des,
                        const Vector3 &acc_des);
    // Update Jt_
    bool _UpdateTaskJacobian();
    // Update JtDotQdot_
    bool _UpdateTaskJDotQdot();
    // Additional Update (defined in child classes)
    bool _AdditionalUpdate();
  };

}
