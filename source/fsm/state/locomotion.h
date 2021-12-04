#pragma once

#include <unordered_map>

#include "fsm/legctrl.h"
#include "mpc/mpc.h"
#include "sdquadx/estimate.h"
#include "sdquadx/fsm.h"
#include "sdquadx/model.h"
#include "wbc/wbc.h"
#include "skd/gait.h"
#include "skd/state_des.h"
#include "estimate/contact.h"

namespace sdquadx::fsm {
class StateLocomotion : public StateCtrl {
 public:
  StateLocomotion(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
                  model::Quadruped::SharedPtr const &mquad, drive::DriveCtrl::ConstSharedPtr const &drictrl,
                  estimate::EstimateCtrl::ConstSharedPtr const &estctrl);
  bool OnEnter() override;
  bool OnExit() override;
  bool RunOnce() override;

  State CheckTransition() override;

  TransitionData Transition(const State next) override;

  State GetState() const override { return State::Locomotion; }

 private:
  // Parses contact specific controls to the leg controller
  bool Step();
  bool SafeCheck();
  bool RunMpcIfNeeded();

  std::unordered_map<drive::State, State> state_trans_;

  Options::ConstSharedPtr const opts_;
  LegCtrl::SharedPtr const legctrl_;
  drive::DriveCtrl::ConstSharedPtr const drictrl_;
  estimate::EstimateCtrl::ConstSharedPtr const estctrl_;

  skd::StateDes::Ptr state_des_;
  mpc::Mpc::Ptr mpc_;
  wbc::Wbc::Ptr wbc_;
  wbc::InData wbc_data_;

  std::unordered_map<drive::Gait, skd::Gait::SharedPtr> gait_skds_;
  long long iter_counter_ = 0;

  std::shared_ptr<estimate::Contact> estcontact_;
};
}  // namespace sdquadx::fsm
