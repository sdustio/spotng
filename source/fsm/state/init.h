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
  State CheckTransition() override;
  TransitionData Transition(const State next) override;

  State GetState() const override { return State::Init; }

 private:
  bool SetJPosInterPts(int const curr_iter, int const max_iter, int const leg, SdVector3f const &ini,
                       SdVector3f const &fin);
  int iter_ = 0;
  bool first_run_ = true;
  std::unordered_map<drive::State, State> state_trans_;

  Options::ConstSharedPtr opts_;
  leg::LegCtrl::SharedPtr legctrl_;
  drive::DriveCtrl::ConstSharedPtr drictrl_;

  JPosVectorf initial_jpos_;
};

}  // namespace sdquadx::fsm
