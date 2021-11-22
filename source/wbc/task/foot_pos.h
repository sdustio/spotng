#pragma once

#include "wbc/task.h"

namespace sdquadx::wbc {
class TaskFootPos : public Task {
 public:
  TaskFootPos(SdVector3f const &kp, SdVector3f const &kd, model::Quadruped::ConstSharedPtr const &quad, int leg);
  bool UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                  SdVector3f const &xdd_des) override;

 private:
  model::Quadruped::ConstSharedPtr const mquad_;
  int const leg_;
};

}  // namespace sdquadx::wbc
