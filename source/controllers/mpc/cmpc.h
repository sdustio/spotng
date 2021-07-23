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
    constexpr static int max_gait_segments = 36;
    constexpr static double big_num = 5e10;
  };

  class QPSolver
  {
  public:
    void Setup(double dt, int horizonLen, double mu, double f_max);
    void ResizeQPMats();
    void SolveQP(double x_drag, const Vector3 &p, const Vector3 &v, const dynamics::Quat &q, const Vector3 &w,
                 const std::array<double, 12> &r, double yaw, std::array<double, 12> &weights,
                 const std::array<double, 12 * 36> &state_trajectory, double alpha, const std::vector<int> &gait);
    const VectorX &GetSolution() { return qsoln; }

  private:
    double dt;
    double mu;
    double f_max;
    int horizon = 0;
    double m = 10.5;

    Eigen::Matrix<double, Eigen::Dynamic, 13> A_qp;
    MatrixX B_qp;
    MatrixX S, eye_12h;
    VectorX X_d;

    MatrixX qH, qA;
    VectorX qg, qlb, qub;
    std::array<char, 250> var_elim = {};
    std::array<char, 250> con_elim = {};

    VectorX qsoln;
  };

  class CMpc : public Mpc
  {
  public:
    CMpc(double _dt, int _iterations_between_mpc);
    bool Init() override;
    bool Run(WbcData &data, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) override;

  private:
    void updateMPCIfNeeded(std::array<Vector3, 4> &out, const std::vector<int> &mpcTable, const StateCmdPtr &cmd, const est::StateEstPtr &est, const Vector3 &v_des_world);
    void solveMPC(std::array<Vector3, 4> &out, const std::vector<int> &mpcTable, const est::StateEstPtr &est);

    double dt;
    double dtMPC;
    int horizonLength = 10;
    int iterationsBetweenMPC;
    int iterationCounter = 0;

    std::array<FootSwingTrajectory, 4> footSwingTrajectories;
    std::array<bool, 4> firstSwing;
    std::array<double, 4> swingTimes;
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

    QPSolver qpsolver_;
    // RobotState qs_;

    std::unordered_map<Gait, GaitSkdPtr> gait_map_;
  };
}
