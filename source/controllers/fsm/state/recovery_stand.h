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

    enum class Flag
    {
      StandUp,
      FoldLegs,
      RollOver
    };
    bool UpsideDown();

    void StandUp(const int curr_iter);
    void FoldLegs(const int curr_iter);
    void RollOver(const int curr_iter);

    void SetJPosInterPts(
        const int curr_iter, int max_iter, int leg,
        const Vector3d &ini, const Vector3d &fin);

    void JointPDControl(int leg, const Vector3d &qDes, const Vector3d &qdDes);

    std::map<robot::Mode, State> state_trans_;
    std::map<Flag, void (StateRecoveryStand::*)(const int)> flag_dispatch_;

    int iter_ = 0;

    Flag flag_ = Flag::FoldLegs;

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
