#include "fsm/state/init.h"

#include "spdlog/spdlog.h"

namespace sdquadx::fsm {

StateInit::StateInit(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                     drive::DriveCtrl::SharedPtr const &drictrl)
    : state_trans_{{drive::State::Init, State::Init},
                   {drive::State::RecoveryStand, State::RecoveryStand},
                   {drive::State::Locomotion, State::RecoveryStand},
                   {drive::State::BalanceStand, State::RecoveryStand}},
      drictrl_(drictrl) {}

bool StateInit::OnEnter() {
  // nothing
  spdlog::debug("Enter State Init!!!");
  return true;
}
bool StateInit::OnExit() {
  // nothing
  return true;
}

bool StateInit::RunOnce() { return true; }

TransitionData StateInit::Transition([[maybe_unused]] const State next) { return TransitionData{true}; }
}  // namespace sdquadx::fsm
