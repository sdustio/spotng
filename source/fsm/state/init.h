#pragma once

#include <unordered_map>

#include "sdquadx/fsm.h"
#include "sdquadx/leg.h"
#include "sdquadx/options.h"

namespace sdquadx::fsm {
class StateInit : public StateCtrl {
 public:
  StateInit(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                     drive::DriveCtrl::SharedPtr const &drictrl);

  bool OnEnter() override;
  bool OnExit() override;
  bool RunOnce() override;
  State CheckTransition() override { return state_trans_[drictrl_->GetState()]; }
  TransitionData Transition(const State next) override;

  State GetState() const override { return State::Init; }

 private:
  std::unordered_map<drive::State, State> state_trans_;

  drive::DriveCtrl::SharedPtr drictrl_;
};

}  // namespace sdquadx::fsm
