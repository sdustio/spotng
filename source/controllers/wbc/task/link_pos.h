#pragma once

#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{
  class TaskLinkPos : public Task
  {
  public:
    TaskLinkPos(const dynamics::FBModelPtr &model, size_t linkid, bool virtual_depend = true);

  private:
    // Update op_cmd_
    bool _UpdateCommand(const VectorXd &pos_des, const VectorXd &vel_des,
                        const VectorXd &acc_des) { return true; }
    // Update Jt_
    bool _UpdateTaskJacobian() { return true; }
    // Update JtDotQdot_
    bool _UpdateTaskJDotQdot() { return true; }
    // Additional Update (defined in child classes)
    bool _AdditionalUpdate() { return true; }

    size_t link_idx_;
    bool virtual_depend_;
  };

}