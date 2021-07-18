#pragma once

#include "sdrobot/controllers/fsm.h"
#include "sdrobot/controllers/wbc.h"
#include "sdrobot/controllers/mpc.h"

namespace sdrobot::ctrl::fsm
{
  class StateLocomotion : public StateCtrl
  {
  public:
    StateLocomotion(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;

    State CheckTransition() override;

    TransitionData Transition(const State next) override;

    State GetState() const override { return State::Locomotion; }

    bool NeedCheckSafeOrientation() const override { return true; }
    bool NeedCheckForceFeedForward() const override { return true; }

  private:
    // Parses contact specific controls to the leg controller
    void LocomotionControlStep();

    bool locomotionSafe();

    std::unordered_map<robot::Mode, State> state_trans_;

    MpcPtr cMPCOld;
    WbcPtr _wbc_ctrl;
    WbcData _wbc_data;
    // Footstep locations for next step
    // Matrix3x4 footstepLocations;

    Matrix3 kp_stance, kd_stance;
  };

} // namespace sdrobot::ctrl::fsm
