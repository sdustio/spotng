#pragma once

#include <unordered_map>

#include "sdrobot/controllers/mpc.h"
#include "controllers/mpc/gait.h"
#include "controllers/mpc/foot_swing.h"

namespace sdrobot::ctrl::mpc
{

  struct Params
  {
    constexpr static int bonus_swing = 0;
  };

  class CMpc : public Mpc
  {
  public:
    CMpc(double _dt, int _iterations_between_mpc);
    bool Init() override;
    bool Run(WbcData &data, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) override;

  private:
    void recompute_timing(int iterations_per_mpc);
    void updateMPCIfNeeded(const std::vector<int> &mpcTable);

    double dt;
    double dtMPC;
    int iterationsBetweenMPC;
    int horizonLength = 10;
    int iterationCounter = 0;

    std::array<FootSwingTrajectory, 4> footSwingTrajectories;
    std::array<bool, 4> firstSwing;
    Vector4 swingTimes;
    std::array<double, 4> swingTimeRemaining;

    double _body_height;

    Vector3 world_position_desired;
    Vector3 rpy_int;
    Vector3 rpy_comp;
    std::array<Vector3, 4> pFoot;

    std::array<double, 6> stand_traj;

    Gait current_gait_;
    bool firstRun = false;

    Matrix3 Kp, Kd, Kp_stance, Kd_stance;

    std::unordered_map<Gait, GaitSkdPtr> gait_map_;
  };
}
