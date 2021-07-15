#pragma once

#include "sdrobot/controllers/wbc.h"

namespace sdrobot::ctrl::wbc
{
  class TaskLinkPos : public Task
  {
  public:
    TaskLinkPos(const dynamics::FBModelPtr &model, size_t linkid, bool virtual_depend = true);

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

    size_t link_idx_;
    bool virtual_depend_;
  };

}
