#pragma once

#include <unordered_map>

#include "spotng/estimate.h"
#include "spotng/fsm.h"
#include "spotng/interface.h"
#include "spotng/model.h"
#include "fsm/legctrl.h"

namespace spotng::fsm {
class FiniteStateMachineImpl : public FiniteStateMachine {
 public:
  FiniteStateMachineImpl(Options::ConstSharedPtr const &opts, interface::Leg::SharedPtr const &legitf,
                         model::Quadruped::SharedPtr const &mquad, drive::DriveCtrl::SharedPtr const &drictrl,
                         estimate::EstimateCtrl::SharedPtr const &estctrl);

  StateCtrl::SharedPtr const &GetStateCtrl(State const state) override;

  bool RunOnce() override;

 private:
  /**
   * Initialize the Control Fsm with the default settings. SHould be set to
   * Passive state and Normal operation mode.
   */
  bool PreCheck();
  bool PostCheck();

  LegCtrl::SharedPtr legctrl_;

  std::unordered_map<State, StateCtrl::SharedPtr> state_ctrls_;

  StateCtrl::SharedPtr current_state_ctrl_;
  StateCtrl::SharedPtr next_state_ctrl_;

  State next_state_;
  OperatingMode opmode_;

  TransitionData transition_data_;
};

}  // namespace spotng::fsm
