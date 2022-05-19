#pragma once

#include "wbc/task.h"

namespace sdengine::wbc {
class TaskBodyOri : public Task {
 public:
  TaskBodyOri(SdVector3f const &kp, SdVector3f const &kd);
  bool UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                  SdVector3f const &xdd_des) override;
};

}  // namespace sdengine::wbc
