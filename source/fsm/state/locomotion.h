#pragma once

#include <unordered_map>

#include "mpc/mpc.h"
#include "sdrobot/estimate.h"
#include "sdrobot/fsm.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"
#include "wbc/wbc_ctrl.h"

namespace sdrobot::fsm {
class StateLocomotion : public StateCtrl {
 public:
  StateLocomotion(Options const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                  model::Quadruped::SharedPtr const &mquad,
                  drive::DriveCtrl::SharedPtr const &drictrl,
                  estimate::EstimateCtrl::SharedPtr const &estctrl);
  bool OnEnter() override;
  bool OnExit() override;
  bool RunOnce() override;

  State CheckTransition() override;

  TransitionData Transition(const State next) override;

  State GetState() const override { return State::Locomotion; }

  bool NeedCheckSafeOrientation() const override { return true; }
  bool NeedCheckForceFeedForward() const override { return true; }

 private:
  // Parses contact specific controls to the leg controller
  bool LocomotionControlStep();

  bool locomotionSafe();

  std::unordered_map<drive::State, State> state_trans_;

  leg::LegCtrl::SharedPtr legctrl_;
  model::Quadruped::ConstSharedPtr mquad_;
  drive::DriveCtrl::ConstSharedPtr drictrl_;
  estimate::EstimateCtrl::ConstSharedPtr estctrl_;

  wbc::WbcCtrl::Ptr wbc_;
  wbc::InData wbc_data_;
  mpc::Mpc::Ptr mpc_;
};
}  // namespace sdrobot::fsm
