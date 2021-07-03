#include "controllers/wbc/task/link_pos.h"

namespace sd::ctrl::wbc
{
  TaskLinkPos::TaskLinkPos(const dynamics::FBModelPtr& model): Task(3, model)
  {
    for (size_t i = 0; i < 3; i++)
    {
      _Kp[i] = robot::DynamicsAttrs::kp_foot[i];
      _Kd[i] = robot::DynamicsAttrs::kd_foot[i];
    }
  }
}
