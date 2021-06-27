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

    void _StandUp(const int curr_iter);
    void _FoldLegs(const int curr_iter);
    void _RollOver(const int curr_iter);

    void _SetJPosInterPts(
        const int curr_iter, int max_iter, int leg,
        const Vector3d &ini, const Vector3d &fin);

    void jointPDControl(int leg, const Vector3d &qDes, const Vector3d &qdDes);

    std::array<State, size_t(robot::Mode::Count_)> state_trans_;
    std::array<void (StateRecoveryStand::*)(const int), 3> flag_dispatch_;

    int _state_iter = 0;

    static constexpr int StandUp = 0;
    static constexpr int FoldLegs = 1;
    static constexpr int RollOver = 2;
    int _flag = FoldLegs;

    // JPos
    Vector3d fold_jpos[4];
    Vector3d stand_jpos[4];
    Vector3d rolling_jpos[4];
    Vector3d initial_jpos[4];

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

    // Create the cartesian P gain matrix
    Matrix3d kpMat;

    // Create the cartesian D gain matrix
    Matrix3d kdMat;
  };

} // namespace sd::ctrl::fsm
