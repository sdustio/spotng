#pragma once

#include <memory>
#include <vector>

#include "wbc/task.h"
#include "wbc/wbc.h"

namespace sdquadx::wbc {
class Wbic : public Wbc {
 public:
  Wbic(Options::ConstSharedPtr const &opts, model::Quadruped::SharedPtr const &quad, fpt_t weight = 0.1);
  bool RunOnce(InData const &wbcdata, estimate::State const &estdata, leg::LegCtrl::SharedPtr const &legctrl) override;

 private:
  bool _ContactTaskUpdate(InData const &wbcdata, estimate::State const &estdata);
  bool _ComputeWBC();
  bool _UpdateLegCMD(leg::LegCtrl::SharedPtr const &legctrl);
  bool _CleanUp();

  Options::ConstSharedPtr const opts_;
  model::Quadruped::SharedPtr const mquad_;

  std::vector<Task::ConstSharedPtr> task_list_;
  std::vector<Task::ConstSharedPtr> contact_list_;

  Task::SharedPtr body_pos_task_;
  Task::SharedPtr body_ori_task_;
  std::array<Task::SharedPtr, consts::model::kNumLeg> foot_task_;
  std::array<Task::SharedPtr, consts::model::kNumLeg> foot_contact_;

  SdVector12f tau_ff_;
  SdVector12f q_cmd_;
  SdVector12f qd_cmd_;

  fpt_t const weight_q_;
  fpt_t const weight_f_ = 1.;
};
}  // namespace sdquadx::wbc
