#pragma once

#include <unordered_map>

#include "sdrobot/fsm.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"

namespace sdrobot::fsm
{
  namespace opts
  {
    constexpr inline int const fold_ramp_iter = 1000;
    constexpr inline int const fold_settle_iter = 1000;

    constexpr inline int const standup_ramp_iter = 500;
    constexpr inline int const standup_settle_iter = 500;

    // 0.5 kHz
    constexpr inline int const rollover_ramp_iter = 150;
    constexpr inline int const rollover_settle_iter = 150;
    // iteration setup
    // constexpr inline int const rollover_ramp_iter = 300;
    // constexpr inline int const rollover_settle_iter = 300;

    // JPos
    constexpr inline std::array<SdVector3f, 4> const fold_jpos = {
        SdVector3f{-0.0, -1.4, 2.7},
        SdVector3f{0.0, -1.4, 2.7},
        SdVector3f{-0.0, -1.4, 2.7},
        SdVector3f{0.0, -1.4, 2.7}};
    constexpr inline std::array<SdVector3f, 4> const stand_jpos = {
        SdVector3f{0., -.8, 1.6},
        SdVector3f{0., -.8, 1.6},
        SdVector3f{0., -.8, 1.6},
        SdVector3f{0., -.8, 1.6}};
    constexpr inline std::array<SdVector3f, 4> const rolling_jpos = {
        SdVector3f{1.5, -1.6, 2.77},
        SdVector3f{1.3, -3.1, 2.77},
        SdVector3f{1.5, -1.6, 2.77},
        SdVector3f{1.3, -3.1, 2.77}};

    // 对角线矩阵，row major == column major
    constexpr inline SdMatrix3f const kp_mat = {80, 0, 0, 0, 80, 0, 0, 0, 80};
    constexpr inline SdMatrix3f const kd_mat = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // kp_mat << 120, 0, 0, 0, 120, 0, 0, 0, 120;
    // kd_mat << 4, 0, 0, 0, 4, 0, 0, 0, 4;
  }

  class StateRecoveryStand : public StateCtrl
  {
  public:
    StateRecoveryStand(
        leg::LegCtrl::SharedPtr const &legctrl,
        drive::DriveCtrl::SharedPtr const &drictrl,
        estimate::EstimateCtrl::SharedPtr const &estctrl);

    void OnEnter() override;
    void OnExit() override;
    bool RunOnce() override;
    State CheckTransition() override
    {
      return state_trans_[drictrl_->GetState()];
    }
    TransitionData Transition(const State next) override;

    State GetState() const override { return State::RecoveryStand; }

  private:
    enum class Flag : uint8_t
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
        int const curr_iter, int const max_iter, int const leg,
        SdVector3f const &ini, SdVector3f const &fin);

    void JointPDControl(int const leg, SdVector3f const &qDes, SdVector3f const &qdDes);

    int iter_ = 0;

    std::unordered_map<drive::State, State> state_trans_;
    std::unordered_map<Flag, void (StateRecoveryStand::*)(const int)> flag_dispatch_;

    Flag flag_ = Flag::FoldLegs;

    leg::LegCtrl::SharedPtr legctrl_;
    drive::DriveCtrl::SharedPtr drictrl_;
    estimate::EstimateCtrl::SharedPtr estctrl_;

    std::array<SdVector3f, 4> initial_jpos_;
  };
}
