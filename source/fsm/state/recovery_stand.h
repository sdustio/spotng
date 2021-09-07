#pragma once

#include <unordered_map>

#include "sdquadx/estimate.h"
#include "sdquadx/fsm.h"
#include "sdquadx/leg.h"
#include "sdquadx/model.h"
#include "sdquadx/options.h"

namespace sdquadx::fsm {
class StateRecoveryStand : public StateCtrl {
 public:
  StateRecoveryStand(Options::ConstSharedPtr const &opts, leg::LegCtrl::SharedPtr const &legctrl,
                     drive::DriveCtrl::SharedPtr const &drictrl, estimate::EstimateCtrl::SharedPtr const &estctrl);

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

  bool SetJPosInterPts(int const curr_iter, int const max_iter, int const leg, SdVector3f const &ini,
                       SdVector3f const &fin, SdVector3f const &kp, SdVector3f const &kd);

  int iter_ = 0;

  std::unordered_map<drive::State, State> state_trans_;
  std::unordered_map<Flag, bool (StateRecoveryStand::*)()> flag_dispatch_;

  Flag flag_ = Flag::FoldLegs;

  Options::ConstSharedPtr opts_;
  leg::LegCtrl::SharedPtr legctrl_;
  drive::DriveCtrl::ConstSharedPtr drictrl_;
  estimate::EstimateCtrl::ConstSharedPtr estctrl_;

  JPosVectorf initial_jpos_;
};
}  // namespace sdquadx::fsm
