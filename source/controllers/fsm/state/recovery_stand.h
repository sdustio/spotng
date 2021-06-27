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
    bool UpsideDown();

    void StandUp(const int curr_iter);
    void FoldLegs(const int curr_iter);
    void RollOver(const int curr_iter);

    void SetJPosInterPts(
        const int curr_iter, int max_iter, int leg,
        const Vector3d &ini, const Vector3d &fin);

    void JointPDControl(int leg, const Vector3d &qDes, const Vector3d &qdDes);

    std::array<State, size_t(robot::Mode::Count_)> state_trans_;
    std::array<void (StateRecoveryStand::*)(const int), 3> flag_dispatch_;

    int _state_iter = 0;

    static constexpr int flag_stand_up_ = 0;
    static constexpr int flag_fold_legs_ = 1;
    static constexpr int flag_roll_over_ = 2;
    int _flag = flag_fold_legs_;

    // JPos
    Vector3d fold_jpos_[4];
    Vector3d stand_jpos_[4];
    Vector3d rolling_jpos_[4];
    Vector3d initial_jpos_[4];

    Vector3d f_ff_;

    const int fold_ramp_iter_ = 1000;
    const int fold_settle_iter_ = 1000;

    const int standup_ramp_iter_ = 500;
    const int standup_settle_iter_ = 500;

    // 0.5 kHz
    const int rollover_ramp_iter_ = 150;
    const int rollover_settle_iter_ = 150;
    // iteration setup
    //  const int rollover_ramp_iter_ = 300;
    //  const int rollover_settle_iter_ = 300;

    // Create the cartesian P gain matrix
    Matrix3d kp_mat_;

    // Create the cartesian D gain matrix
    Matrix3d kd_mat_;
  };

} // namespace sd::ctrl::fsm
