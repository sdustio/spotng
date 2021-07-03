#include "controllers/wbc/task/body_ori.h"

namespace sd::ctrl::wbc
{
  TaskBodyOri::TaskBodyOri(const dynamics::FBModelPtr& model): Task(3, model)
  {
    for (size_t i = 0; i < 3; i++)
    {
      _Kp[i] = robot::DynamicsAttrs::kp_ori[i];
      _Kd[i] = robot::DynamicsAttrs::kd_ori[i];
    }
  }
}
