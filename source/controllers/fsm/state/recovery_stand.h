#pragma once

#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  class StateRecoveryStand : public StateCtrl
  {
  public:
    StateRecoveryStand(LegPtr &cleg, const StateCmdPtr &cmd, const est::StateEstPtr &est);
    void OnEnter() override;
    void OnExit() override;
    bool Run() override;
    State CheckTransition(const StateCmdPtr &cmd) override;
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::RecoveryStand; }

  private:
    bool _UpsideDown();

    void _StandUp(const int iter);
    void _FoldLegs(const int iter);
    void _RollOver(const int iter);

    void _SetJPosInterPts(
        const int curr_iter, size_t max_iter, int leg,
        const Vector3d &ini, const Vector3d &fin);

    std::array<State, size_t(robot::Mode::Count_)> state_trans_;
    std::array<void(StateRecoveryStand::*)(const int), 3> flag_dispatch_;

    int _iter = 0;
    int _motion_start_iter = 0;

    static constexpr int StandUp = 0;
    static constexpr int FoldLegs = 1;
    static constexpr int RollOver = 2;

    unsigned long long _state_iter;
    int _flag = FoldLegs;

    // JPos
    Vector3d fold_jpos[4];
    Vector3d stand_jpos[4];
    Vector3d rolling_jpos[4];
    Vector3d initial_jpos[4];
    Vector3d zero_vec3;

    Vector3d f_ff;

    // iteration setup
    //  const int rollover_ramp_iter = 300;
    //  const int rollover_settle_iter = 300;

    const int fold_ramp_iter = 1000;
    const int fold_settle_iter = 1000;

    const int standup_ramp_iter = 500;
    const int standup_settle_iter = 500;

    // 0.5 kHz
    const int rollover_ramp_iter = 150;
    const int rollover_settle_iter = 150;
  };

} // namespace sd::ctrl::fsm
