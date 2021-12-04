#pragma once

#include <unordered_map>

#include "estimate/contact.h"
#include "fsm/legctrl.h"
#include "sdquadx/fsm.h"
#include "sdquadx/model.h"
#include "wbc/wbc.h"

namespace sdquadx::fsm {
class StateBalanceStand : public StateCtrl {
 public:
  StateBalanceStand(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
                    model::Quadruped::SharedPtr const &mquad, drive::DriveCtrl::ConstSharedPtr const &drictrl,
                    estimate::EstimateCtrl::ConstSharedPtr const &estctrl);
  bool OnEnter() override;
  bool OnExit() override;
  bool RunOnce() override;

  State CheckTransition() override;
  TransitionData Transition(const State next) override;

  State GetState() const override { return State::BalanceStand; }

 private:
  // Parses contact specific controls to the leg controller
  bool SafeCheck();

  std::unordered_map<drive::State, State> state_trans_;

  LegCtrl::SharedPtr const legctrl_;
  drive::DriveCtrl::ConstSharedPtr const drictrl_;
  estimate::EstimateCtrl::ConstSharedPtr const estctrl_;

  wbc::Wbc::Ptr wbc_;
  wbc::InData wbc_data_;

  SdVector3f ini_body_pos_;
  SdVector3f ini_body_rpy_;
  fpt_t body_weight_;

  std::shared_ptr<estimate::Contact> estcontact_;
};
}  // namespace sdquadx::fsm
