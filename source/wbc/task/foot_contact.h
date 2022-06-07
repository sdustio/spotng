#pragma once

#include <memory>

#include "spotng/model.h"
#include "wbc/task.h"

namespace spotng::wbc {
class TaskFootContact : public Task {
 public:
  TaskFootContact(model::Quadruped::ConstSharedPtr const &quad, int leg);
  bool UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                  SdVector3f const &xdd_des) override;

 private:
  model::Quadruped::ConstSharedPtr const mquad_;
  int leg_;
};

}  // namespace spotng::wbc
