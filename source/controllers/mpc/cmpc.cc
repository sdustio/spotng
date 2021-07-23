#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include "sdrobot/dynamics/rotation.h"
#include "controllers/mpc/cmpc.h"

namespace sdrobot::ctrl::mpc
{
  using Eigen::Vector4i;

  bool near_zero(double a)
  {
    return (a < .01 && a > -.01);
  }

  bool near_one(double a)
  {
    return near_zero(a - 1);
  }

  CMpc::CMpc(double _dt, unsigned _iterations_between_mpc) : dt(_dt),
                                                             dtMPC(_dt * _iterations_between_mpc),
                                                             iterationsBetweenMPC(_iterations_between_mpc),
                                                             rpy_int(Vector3::Zero())
  {
    firstSwing.fill(true);
    gait_map_ = {
        {Gait::Trot, std::make_shared<OffsetDurationGait>(horizonLength, Vector4i(0, 5, 5, 0), Vector4i(5, 5, 5, 5), "Trot")},
        {Gait::SlowTrot, std::make_shared<OffsetDurationGait>(int(horizonLength * 1.2), Vector4i(0, 6, 6, 0), Vector4i(6, 6, 6, 6), "SlowTrot")},
        {Gait::FlyingTrot, std::make_shared<OffsetDurationGait>(horizonLength, Vector4i(0, 5, 5, 0), Vector4i(4, 4, 4, 4), "FlyingTrot")},
        {Gait::Walk, std::make_shared<OffsetDurationGait>(int(horizonLength * 1.6), Vector4i(0, 8, 4, 12), Vector4i(12, 12, 12, 12), "Walk")},
        {Gait::Bound, std::make_shared<OffsetDurationGait>(horizonLength, Vector4i(5, 5, 0, 0), Vector4i(5, 5, 5, 5), "Bound")},
        {Gait::Pronk, std::make_shared<OffsetDurationGait>(horizonLength, Vector4i(0, 0, 0, 0), Vector4i(4, 4, 4, 4), "Pronk")}};
    // TODO
    // setup_problem(dtMPC, horizonLength, 0.4, 120);
  }

  bool CMpc::Run(WbcData &wbcdata, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est)
  {
    const auto &cmd_des = cmd->GetStateDes();

    auto _body_height = 0.3 + cmd_des(StateIdx::pos_z);

    const auto &seResult = est->GetData();

    // some first time initialization
    if (firstRun)
    {
      stand_traj[0] = seResult.position[0];
      stand_traj[1] = seResult.position[1];
      stand_traj[2] = _body_height; // ?? 0.21;
      stand_traj[3] = 0;
      stand_traj[4] = 0;
      stand_traj[5] = seResult.rpy[2];
      world_position_desired[0] = stand_traj[0];
      world_position_desired[1] = stand_traj[1];
      world_position_desired[2] = stand_traj[2]; //?? seResult.rpy[2];

      for (size_t i = 0; i < 4; i++)
      {

        footSwingTrajectories[i].setHeight(0.05);
        footSwingTrajectories[i].setInitialPosition(pFoot[i]);
        footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      }
      firstRun = false;
    }

    auto cmd_gait = cmd->GetGait();
    GaitSkdPtr gait_skd = gait_map_[cmd_gait];

    gait_skd->setIterations(iterationsBetweenMPC, iterationCounter);

    if (_body_height < 0.02)
    {
      _body_height = 0.3;
    }

    if (cmd_gait == Gait::FlyingTrot)
      _body_height = 0.31;

    // integrate position setpoint
    Vector3 v_des_robot = cmd_des.segment<3>(StateIdx::vel_x);
    Vector3 v_des_world = seResult.rot_body.transpose() * v_des_robot;
    const auto &v_world = seResult.v_world;

    //Integral-esque pitche and roll compensation
    if (fabs(v_world[0]) > .02) //avoid dividing by zero
    {
      rpy_int[1] += 5 * dt * (cmd_des(StateIdx::angle_p) - seResult.rpy[1]) / v_world[0];
    }
    if (fabs(v_world[1]) > 0.01)
    {
      rpy_int[0] += dt * (cmd_des(StateIdx::angle_r) - seResult.rpy[0]) / v_world[1];
    }

    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);

    for (size_t i = 0; i < 4; i++)
    {
      pFoot[i] = seResult.position +
                 seResult.rot_body.transpose() * (quad->GetHipLocation(i) +
                                                  cleg->GetDatas()[i].p);
    }

    world_position_desired += dt * v_des_world;
    world_position_desired[2] = _body_height;

    // foot placement
    for (size_t l = 0; l < 4; l++)
      swingTimes[l] = gait_skd->getCurrentSwingTime(dtMPC, l);

    double side_sign[4] = {-1, 1, -1, 1};
    //  double interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
    double interleave_y[4] = {0.0, 0.0, 0.02, -0.02};
    //double interleave_gain = -0.13;
    double interleave_gain = 0; //-0.2;
    //double v_abs = std::fabs(seResult.vBody[0]);
    double v_abs = std::fabs(v_des_robot[0]);

    for (size_t i = 0; i < 4; i++)
    {
      if (firstSwing[i])
      {
        swingTimeRemaining[i] = swingTimes[i];
      }
      else
      {
        swingTimeRemaining[i] -= dt;
      }
      //if(firstSwing[i]) {
      //footSwingTrajectories[i].setHeight(.05);

      footSwingTrajectories[i].setHeight(cmd->GetStepHeight()); //.125);

      //    Vector3 offset(0.05, side_sign[i] * .062, 0);
      Vector3 offset(0, side_sign[i] * .072, 0);
      //     Vector3 offset(0, side_sign[i] * .075, 0);

      if (i < 2)
        offset[0] = 0; //0.03;
      else
        offset[0] = -0.02; //0.02;//

      //      offset[0]=seResult.vBody[0]*0.01;

      if (cmd_gait == Gait::Walk) //walk gait
      {
        if (i == 0)
          offset(1) = -0.085 * (1 - fabs(v_des_robot[0]) / 2.0);
        else if (i == 1)
          offset(1) = 0.085 * (1 - fabs(v_des_robot[0]) / 2.0);
      }

      Vector3 pRobotFrame = (quad->GetHipLocation(i) + offset);

      pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
      double stance_time = gait_skd->getCurrentStanceTime(dtMPC, i);
      Vector3 pYawCorrected =
          dynamics::CoordinateRot(dynamics::CoordinateAxis::Z, -cmd_des(StateIdx::rate_y) * stance_time / 2) * pRobotFrame;

      Vector3 Pf = seResult.position + seResult.rot_body.transpose() * (pYawCorrected + v_des_robot * swingTimeRemaining[i]);

      //+ v_world * swingTimeRemaining[i];

      double p_rel_max = 0.35;
      //    double p_rel_max = 0.3f;

      // Using the estimated velocity is correct
      //Vector3 v_des_robot_world = seResult.rot_body.transpose() * v_des_robot;
      double pfx_rel = v_world[0] * (.5 + Params::bonus_swing) * stance_time +
                       .1 * (v_world[0] - v_des_world[0]) +
                       (0.5 * seResult.position[2] / 9.81) * (v_world[1] * cmd_des(StateIdx::rate_y));

      if (fabs(pfx_rel) > p_rel_max)
        printf("!!!!!!!!!!!!!!!!out of the max step\n");

      double pfy_rel = v_world[1] * .5 * stance_time * dtMPC +
                       .09 * (v_world[1] - v_des_world[1]) +
                       (0.5 * seResult.position[2] / 9.81) * (-v_world[0] * cmd_des(StateIdx::rate_y));
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel;
      Pf[2] = -0.01; //0;//-0.003; //
      //Pf[2] = 0.0;
      footSwingTrajectories[i].setFinalPosition(Pf);
    }

    // calc gait
    iterationCounter++;

    // gait
    Vector4 swingStates = gait_skd->getSwingState();

    updateMPCIfNeeded(wbcdata.Fr_des, gait_skd->getMpcTable(), cmd, est, v_des_world);

    //  StateEstimator* se = hw_i->state_estimator;
    for (size_t foot = 0; foot < 4; foot++)
    {
      double swingState = swingStates[foot];
      if (swingState > 0) // foot is in swing
      {
        if (firstSwing[foot])
        {
          firstSwing[foot] = false;
          footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        }

        footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

        Vector3 pDesFootWorld = footSwingTrajectories[foot].getPosition();
        Vector3 vDesFootWorld = footSwingTrajectories[foot].getVelocity();

        // Update for WBC
        wbcdata.pFoot_des[foot] = pDesFootWorld;
        wbcdata.vFoot_des[foot] = vDesFootWorld;
        wbcdata.aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      }
      else // foot is in stance
      {
        firstSwing[foot] = true;

        Vector3 pDesFootWorld = footSwingTrajectories[foot].getPosition();
        Vector3 vDesFootWorld = footSwingTrajectories[foot].getVelocity();
        Vector3 pDesLeg = seResult.rot_body * (pDesFootWorld - seResult.position) - quad->GetHipLocation(foot);
        Vector3 vDesLeg = seResult.rot_body * (vDesFootWorld - v_world);
        //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

        // Stance foot damping
        auto &leg_cmds = cleg->GetCmdsForUpdate();
        leg_cmds[foot].p_des = pDesLeg;
        leg_cmds[foot].v_des = vDesLeg;
      }
    }

    // Update For WBC
    wbcdata.pBody_des[0] = world_position_desired[0];
    wbcdata.pBody_des[1] = world_position_desired[1];
    wbcdata.pBody_des[2] = world_position_desired[2];

    wbcdata.vBody_des[0] = v_des_world[0];
    wbcdata.vBody_des[1] = v_des_world[1];
    wbcdata.vBody_des[2] = v_des_world[2];

    wbcdata.aBody_des.setZero();

    //  pBody_RPY_des[0] = 0.;
    //  pBody_RPY_des[1] = 0.;
    wbcdata.pBody_RPY_des[0] = cmd_des(StateIdx::angle_r); //data._desiredStateCommand->data.stateDes(3); // pBody_RPY_des[0]*0.9+0.1*seResult.rpy[0]/2.0;//
    wbcdata.pBody_RPY_des[1] = cmd_des(StateIdx::angle_p); //pBody_RPY_des[1]*0.9+0.1*seResult.rpy[1]/2.0;//
    wbcdata.pBody_RPY_des[2] = cmd_des(StateIdx::angle_y);

    wbcdata.vBody_Ori_des[0] = cmd_des(StateIdx::rate_r);
    wbcdata.vBody_Ori_des[1] = cmd_des(StateIdx::rate_p);
    wbcdata.vBody_Ori_des[2] = cmd_des(StateIdx::rate_y);

    wbcdata.contact_state = gait_skd->getContactState();
    // END of WBC Update

    return true;
  }

  bool CMpc::Init()
  {
    return true;
  }

  void CMpc::updateMPCIfNeeded(std::array<Vector3, 4> &out, const std::vector<int> &mpcTable, const StateCmdPtr &cmd, const est::StateEstPtr &est, const Vector3 &v_des_world)
  {
    //iterationsBetweenMPC = 30;
    if ((iterationCounter % iterationsBetweenMPC) != 0)
    {
      return;
    }

    if ((iterationCounter / iterationsBetweenMPC) >= horizonLength)
    {
      iterationCounter = 0;
    }

    const auto &seResult = est->GetData();
    const auto &cmd_des = cmd->GetStateDes();
    const auto &pos = seResult.position;
    const auto &v_world = seResult.v_world;

    Vector3 rpy_comp = Vector3::Zero();
    rpy_comp[1] = v_world[0] * rpy_int[1];
    rpy_comp[0] = v_world[1] * rpy_int[0] * int(cmd->GetGait() != Gait::Pronk); //turn off for pronking
    rpy_comp[2] = cmd_des(StateIdx::angle_y);

    const double max_pos_error = .1;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];

    if (xStart - pos[0] > max_pos_error)
      xStart = pos[0] + max_pos_error;
    if (pos[0] - xStart > max_pos_error)
      xStart = pos[0] - max_pos_error;

    if (yStart - pos[1] > max_pos_error)
      yStart = pos[1] + max_pos_error;
    if (pos[1] - yStart > max_pos_error)
      yStart = pos[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    double trajInitial[12] = {rpy_comp[0],               // 0
                              rpy_comp[1],               // 1
                              rpy_comp[2],               // 2
                              world_position_desired[0], // 3
                              world_position_desired[1], // 4
                              world_position_desired[2], // 5
                              cmd_des(StateIdx::rate_r), // 6
                              cmd_des(StateIdx::rate_p), // 7
                              cmd_des(StateIdx::rate_y), // 8
                              v_des_world[0],            // 9
                              v_des_world[1],            // 10
                              v_des_world[2]};           // 11

    for (size_t i = 0; i < horizonLength; i++)
    {
      for (size_t j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      if (i == 0) // start at current position  TODO consider not doing this
      {
        //trajAll[3] = hw_i->state_estimator->se_pBody[0];
        //trajAll[4] = hw_i->state_estimator->se_pBody[1];
        trajAll[2] = seResult.rpy[2];
      }
      else
      {
        trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
        trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
        trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * cmd_des(StateIdx::rate_y);
      }
    }
    solveMPC(out, mpcTable, est);
    //    printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

  void CMpc::solveMPC(std::array<Vector3, 4> &out, const std::vector<int> &mpcTable, const est::StateEstPtr &est)
  {
    const auto &seResult = est->GetData();

    std::array<double, 12> weights = {1.25, 1.25, 10, 2, 2, 50, 0, 0, 0.3, 1.5, 1.5, 0.2};
    //double Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
    double yaw = seResult.rpy[2];
    double alpha = 4e-5; // make setting eventually
    //double alpha = 4e-7; // make setting eventually: DH
    const auto &pos = seResult.position;        //p
    const auto &v_world = seResult.v_world;     //v
    const auto &w_world = seResult.omega_world; //w
    const auto &ori = seResult.orientation;     //q

    std::array<double, 12> r;
    for (int i = 0; i < 12; i++)
      r[i] = pFoot[i % 4][i / 4] - pos[i / 4];

    if (alpha > 1e-4)
    {
      printf("Alpha was set too high (%f) adjust to 1e-5\n", alpha);
      alpha = 1e-5;
    }

    double pz_err = pos[2] - world_position_desired[2];

    Vector3 vxy(v_world[0], v_world[1], 0);

    qpsolver_.Setup(dtMPC, horizonLength, 0.4, 150);

    if (vxy[0] > 0.3 || vxy[0] < -0.3)
    {
      x_comp_integral += Params::cmpc_x_drag * pz_err * dtMPC / vxy[0];
    }

    qpsolver_.SolveQP(x_comp_integral, pos, v_world, ori, w_world, r, yaw, weights, trajAll, alpha, mpcTable);

    auto const &solu = qpsolver_.GetSolution();
    for (int leg = 0; leg < 4; leg++)
    {
      Vector3 f;
      for (int axis = 0; axis < 3; axis++)
        f[axis] = solu(leg * 3 + axis);

      out[leg] = f;
    }
  }

  void QPSolver::SolveQP(double x_drag, const Vector3 &p, const Vector3 &v, const dynamics::Quat &q, const Vector3 &w,
                         const std::array<double, 12> &r, double yaw, std::array<double, 12> &weights,
                         const std::array<double, 12 * 36> &state_trajectory, double alpha, const std::vector<int> &gait)
  {
    Matrix3x4 r_feet;
    for (int rs = 0; rs < 3; rs++)
      for (int c = 0; c < 4; c++)
        r_feet(rs, c) = r[rs * 4 + c];
    Matrix3 I_body, R_yaw;
    auto yc = cos(yaw);
    auto ys = sin(yaw);
    R_yaw << yc, -ys, 0,
        ys, yc, 0,
        0, 0, 1;
    I_body.diagonal() << .07, 0.26, 0.242;

    auto rpy = dynamics::QuatToRPY(q);
    Eigen::Matrix<double, 13, 1> x_0;
    x_0 << rpy(2), rpy(1), rpy(0), p, w, v, -9.8f;

    Matrix3 I_world = R_yaw * I_body * R_yaw.transpose();

    //continuous time state space matrices.
    Eigen::Matrix<double, 13, 13> A_ct;
    Eigen::Matrix<double, 13, 12> B_ct_r;

    A_ct.setZero();
    A_ct(3, 9) = 1.f;
    A_ct(11, 9) = x_drag;
    A_ct(4, 10) = 1.f;
    A_ct(5, 11) = 1.f;

    A_ct(11, 12) = 1.f;
    A_ct.block(0, 6, 3, 3) = R_yaw.transpose();

    B_ct_r.setZero();
    Matrix3 I_inv = I_world.inverse();

    for (int b = 0; b < 4; b++)
    {
      B_ct_r.block<3, 3>(6, b * 3) = I_inv * dynamics::VecToSkewMat(r_feet.col(b));
      B_ct_r.block<3, 3>(9, b * 3) = Matrix3::Identity() / m;
    }

    //QP matrices
    Eigen::Matrix<double, 13, 12> Bdt;
    Eigen::Matrix<double, 13, 13> Adt;

    Eigen::Matrix<double, 25, 25> ABc = Eigen::Matrix<double, 25, 25>::Zero();
    ABc.block<13, 13>(0, 0) = A_ct;
    ABc.block<13, 12>(0, 13) = B_ct_r;
    ABc = dt * ABc;
    Eigen::Matrix<double, 25, 25> expmm = ABc.exp();
    Adt = expmm.block<13, 13>(0, 0);
    Bdt = expmm.block<13, 12>(0, 13);

    Eigen::Matrix<double, 13, 13> powerMats[20];
    powerMats[0].setIdentity();
    for (int i = 1; i < horizon + 1; i++)
    {
      powerMats[i] = Adt * powerMats[i - 1];
    }

    for (int r = 0; r < horizon; r++)
    {
      A_qp.block(13 * r, 0, 13, 13) = powerMats[r + 1]; //Adt.pow(r+1);
      for (int c = 0; c < horizon; c++)
      {
        if (r >= c)
        {
          int a_num = r - c;
          B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
        }
      }
    }

    //weights
    Eigen::Matrix<double, 13, 1> full_weight;
    for (int i = 0; i < 12; i++)
      full_weight(i) = weights[i];
    full_weight(12) = 0.f;
    S.diagonal() = full_weight.replicate(horizon, 1);

    //trajectory
    for (int i = 0; i < horizon; i++)
    {
      for (int j = 0; j < 12; j++)
        X_d(13 * i + j, 0) = state_trajectory[12 * i + j];
    }

    int k = 0;
    for (int i = 0; i < horizon; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        qub(5 * k + 0) = Params::big_num;
        qub(5 * k + 1) = Params::big_num;
        qub(5 * k + 2) = Params::big_num;
        qub(5 * k + 3) = Params::big_num;
        qub(5 * k + 4) = gait[i * 4 + j] * f_max;
        k++;
      }
    }

    double rep_mu = 1. / (mu + kZeroEpsilon);
    Eigen::Matrix<double, 5, 3> f_block;
    f_block << rep_mu, 0, 1.,
        -rep_mu, 0, 1.,
        0, rep_mu, 1.,
        0, -rep_mu, 1.,
        0, 0, 1.;

    for (int i = 0; i < horizon * 4; i++)
    {
      qA.block(i * 5, i * 3, 5, 3) = f_block;
    }

    qH = 2 * (B_qp.transpose() * S * B_qp + alpha * eye_12h);
    qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);

    int nWSR = 100;
    int num_constraints = 20 * horizon;
    int num_variables = 12 * horizon;
    int new_cons = num_constraints;
    int new_vars = num_variables;

    for (int i = 0; i < num_constraints; i++)
      con_elim[i] = 0;

    for (int i = 0; i < num_variables; i++)
      var_elim[i] = 0;

    for (int i = 0; i < num_constraints; i++)
    {
      if (!(near_zero(qlb[i]) && near_zero(qub[i])))
        continue;
      auto c_row = qA.row(i);

      for (int j = 0; j < num_variables; j++)
      {
        if (near_one(c_row[j]))
        {
          new_vars -= 3;
          new_cons -= 5;
          int cs = (j * 5) / 3 - 3;
          var_elim[j - 2] = 1;
          var_elim[j - 1] = 1;
          var_elim[j] = 1;
          con_elim[cs] = 1;
          con_elim[cs + 1] = 1;
          con_elim[cs + 2] = 1;
          con_elim[cs + 3] = 1;
          con_elim[cs + 4] = 1;
        }
      }
    }

    auto var_ind = new int[new_vars];
    auto con_ind = new int[new_cons];
    int vc = 0;

    for (int i = 0; i < num_variables; i++)
    {
      if (!var_elim[i])
      {
        if (vc >= new_vars)
        {
          throw("BAD ERROR 1\n");
        }
        var_ind[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++)
    {
      if (!con_elim[i])
      {
        if (vc >= new_cons)
        {
          throw("BAD ERROR 1\n");
        }
        con_ind[vc] = i;
        vc++;
      }
    }

    auto g_red = new qpOASES::real_t[new_vars];
    auto H_red = new qpOASES::real_t[new_vars * new_vars];
    for (int i = 0; i < new_vars; i++)
    {
      int olda = var_ind[i];
      g_red[i] = qg[olda];
      for (int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
        H_red[i * new_vars + j] = qH(olda, oldb);
      }
    }

    auto A_red = new qpOASES::real_t[new_cons * new_vars];
    for (int con = 0; con < new_cons; con++)
    {
      for (int st = 0; st < new_vars; st++)
      {
        auto cval = qA(con_ind[con], var_ind[st]);
        A_red[con * new_vars + st] = cval;
      }
    }

    auto ub_red = new qpOASES::real_t[new_cons];
    auto lb_red = new qpOASES::real_t[new_cons];
    for (int i = 0; i < new_cons; i++)
    {
      int old = con_ind[i];
      ub_red[i] = qub[old];
      lb_red[i] = qlb[old];
    }

    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    //int_t nWSR = 50000;

    problem_red.init(H_red, g_red, A_red, nullptr, nullptr, lb_red, ub_red, nWSR);

    auto q_red = new qpOASES::real_t[new_vars];
    int rval2 = problem_red.getPrimalSolution(q_red);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");

    vc = 0;
    for (int i = 0; i < num_variables; i++)
    {
      if (var_elim[i])
      {
        qsoln[i] = 0.0;
      }
      else
      {
        qsoln[i] = q_red[vc];
        vc++;
      }
    }

    delete[] var_ind;
    delete[] con_ind;
    delete[] g_red;
    delete[] H_red;
    delete[] A_red;
    delete[] ub_red;
    delete[] lb_red;
    delete[] q_red;
  }

  void QPSolver::Setup(double dt, int horizonLen, double mu, double f_max)
  {
    auto changed = horizon != horizonLen;
    horizon = horizonLen;
    f_max = f_max;
    mu = mu;
    dt = dt;

    if (changed)
    {
      ResizeQPMats();
    }
  }

  void QPSolver::ResizeQPMats()
  {
    A_qp.resize(13*horizon, Eigen::NoChange);
    B_qp.resize(13*horizon, 12*horizon);
    S.resize(13*horizon, 13*horizon);
    X_d.resize(13*horizon, Eigen::NoChange);
    qub.resize(20*horizon, Eigen::NoChange);
    qlb.resize(20*horizon, Eigen::NoChange);
    qA.resize(20*horizon, 12*horizon);
    qH.resize(12*horizon, 12*horizon);
    qg.resize(12*horizon, Eigen::NoChange);
    eye_12h.resize(12*horizon, 12*horizon);
    qsoln.resize(12*horizon, Eigen::NoChange);

    A_qp.setZero();
    B_qp.setZero();
    S.setZero();
    X_d.setZero();
    qub.setZero();
    qlb.setZero();
    qA.setZero();
    qH.setZero();
    qg.setZero();
    eye_12h.setIdentity();
    qsoln.setZero();
  }

}
