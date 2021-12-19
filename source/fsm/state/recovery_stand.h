#pragma once

#include <memory>
#include <unordered_map>

#include "estimate/contact.h"
#include "fsm/legctrl.h"
#include "sdquadx/fsm.h"
#include "sdquadx/model.h"
#include "sdquadx/options.h"

namespace sdquadx::fsm {
class StateRecoveryStand : public StateCtrl {
 public:
  StateRecoveryStand(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
                     drive::DriveCtrl::ConstSharedPtr const &drictrl,
                     estimate::EstimateCtrl::ConstSharedPtr const &estctrl);

  bool OnEnter() override;
  bool OnExit() override;
  bool RunOnce() override;
  State CheckTransition() override;
  TransitionData Transition(const State next) override;

  State GetState() const override { return State::RecoveryStand; }

 private:
  enum class Flag : uint8_t { StandUp, FoldLegs, RollOver };
  bool UpsideDown();

  bool StandUp();
  bool FoldLegs();
  bool RollOver();

  bool SetJPosInterPts(interface::LegCmd &cmd, int const curr_iter, int const max_iter, SdVector3f const &ini,
                       SdVector3f const &fin);

  int iter_ = 0;

  std::unordered_map<drive::State, State> state_trans_;
  std::unordered_map<Flag, bool (StateRecoveryStand::*)()> flag_dispatch_;

  Flag flag_ = Flag::FoldLegs;

  Options::ConstSharedPtr const opts_;
  LegCtrl::SharedPtr const legctrl_;
  drive::DriveCtrl::ConstSharedPtr const drictrl_;
  estimate::EstimateCtrl::ConstSharedPtr const estctrl_;

  std::array<SdVector3f, consts::model::kNumLeg> initial_jpos_;

  std::shared_ptr<estimate::Contact> estcontact_;
};
}  // namespace sdquadx::fsm
