#include "controllers/wbc/task/link_pos.h"

namespace sd::ctrl::wbc
{
  TaskLinkPos::TaskLinkPos(
    const dynamics::FBModelPtr& model, size_t linkid, bool virtual_depend)
    : Task(3, model), link_idx_(linkid), virtual_depend_(virtual_depend)
  {
    for (size_t i = 0; i < 3; i++)
    {
      _Kp[i] = robot::DynamicsAttrs::kp_foot[i];
      _Kd[i] = robot::DynamicsAttrs::kd_foot[i];
    }
  }
}
