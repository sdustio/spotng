#pragma once

#include <unordered_map>

#include "fsm/legctrl.h"
#include "spotng/estimate.h"
#include "spotng/fsm.h"
#include "spotng/options.h"

namespace spotng::fsm {
class StateInit : public StateCtrl {
 public:
  StateInit(Options::ConstSharedPtr const &opts, LegCtrl::SharedPtr const &legctrl,
            drive::DriveCtrl::ConstSharedPtr const &drictrl, estimate::EstimateCtrl::ConstSharedPtr const &estctrl);

  bool OnEnter() override;
  bool OnExit() override;
  bool RunOnce() override;
  State CheckTransition() override;
  TransitionData Transition(const State next) override;

  State GetState() const override { return State::Init; }

 private:
  bool SetJPosInterPts(interface::LegCmd &cmd, int const curr_iter, int const max_iter, SdVector3f const &ini,
                       SdVector3f const &fin);
  int iter_ = 0;
  bool first_run_ = true;
  std::unordered_map<drive::State, State> state_trans_;

  Options::ConstSharedPtr const opts_;
  LegCtrl::SharedPtr const legctrl_;
  drive::DriveCtrl::ConstSharedPtr const drictrl_;
  estimate::EstimateCtrl::ConstSharedPtr const estctrl_;

  std::array<SdVector3f, consts::model::kNumLeg> initial_jpos_;
};

}  // namespace spotng::fsm
