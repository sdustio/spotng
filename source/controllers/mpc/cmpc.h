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
    constexpr static int cmpc_x_drag = 3;
  };

  class CMpc : public Mpc
  {
  public:
    CMpc(double _dt, unsigned _iterations_between_mpc);
    bool Init() override;
    bool Run(WbcData &data, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) override;

  private:
    void updateMPCIfNeeded(std::array<Vector3, 4> &out, const std::vector<int> &mpcTable, const StateCmdPtr &cmd, const est::StateEstPtr &est, const Vector3 &v_des_world);
    void solveDenseMPC(std::array<Vector3, 4> &out, const std::vector<int> &mpcTable, const est::StateEstPtr &est);

    double dt;
    double dtMPC;
    unsigned iterationsBetweenMPC;
    unsigned horizonLength = 10;
    unsigned iterationCounter = 0;

    std::array<FootSwingTrajectory, 4> footSwingTrajectories;
    std::array<bool, 4> firstSwing;
    Vector4 swingTimes;
    std::array<double, 4> swingTimeRemaining;

    Vector3 world_position_desired;
    Vector3 rpy_int;
    std::array<Vector3, 4> pFoot;
    double x_comp_integral = 0;

    std::array<double, 6> stand_traj;

    Gait current_gait_;
    bool firstRun = true;
    std::array<double, 12 * 36> trajAll;

    Matrix3 Kp, Kd, Kp_stance, Kd_stance;

    std::unordered_map<Gait, GaitSkdPtr> gait_map_;
  };
}
