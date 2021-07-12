#include "controllers/wbc/task/body_pos.h"

namespace sdrobot::ctrl::wbc
{
  TaskBodyPos::TaskBodyPos(const dynamics::FBModelPtr& model): Task(3, model)
  {
    for (size_t i = 0; i < 3; i++)
    {
      _Kp[i] = robot::DynamicsAttrs::kp_body[i];
      _Kd[i] = robot::DynamicsAttrs::kd_body[i];
    }
  }
}
