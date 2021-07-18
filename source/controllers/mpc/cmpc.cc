#include "sdrobot/dynamics/rotation.h"
#include "controllers/mpc/cmpc.h"

namespace sdrobot::ctrl::mpc
{
  using Eigen::Vector4i;

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

  bool CMpc::Run(WbcData &data_, LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est)
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
    Vector4 contactStates = gait_skd->getContactState();
    //  std::cout<<"contactStates:"<<contactStates<<std::endl;
    Vector4 swingStates = gait_skd->getSwingState();

    updateMPCIfNeeded(gait_skd->getMpcTable(), cmd, est, v_des_world);

    //  StateEstimator* se = hw_i->state_estimator;
    Vector4 se_contactState(0, 0, 0, 0);
    for (size_t foot = 0; foot < 4; foot++)
    {
      double contactState = contactStates[foot];
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
        data_.pFoot_des[foot] = pDesFootWorld;
        data_.vFoot_des[foot] = vDesFootWorld;
        data_.aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
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
        leg_cmds[foot].kp_cartesian.setZero();
        leg_cmds[foot].kd_cartesian << 14, 0, 0,
            0, 14, 0,
            0, 0, 14;

        se_contactState[foot] = contactState;

        // Update for WBC
        //Fr_des[foot] = -f_ff[foot];
      }
    }

    // TODO check
    // data._stateEstimator->setContactPhase(se_contactState);

    // Update For WBC
    data_.pBody_des[0] = world_position_desired[0];
    data_.pBody_des[1] = world_position_desired[1];
    data_.pBody_des[2] = world_position_desired[2];

    data_.vBody_des[0] = v_des_world[0];
    data_.vBody_des[1] = v_des_world[1];
    data_.vBody_des[2] = v_des_world[2];

    data_.aBody_des.setZero();

    //  pBody_RPY_des[0] = 0.;
    //  pBody_RPY_des[1] = 0.;
    data_.pBody_RPY_des[0] = cmd_des(StateIdx::angle_r); //data._desiredStateCommand->data.stateDes(3); // pBody_RPY_des[0]*0.9+0.1*seResult.rpy[0]/2.0;//
    data_.pBody_RPY_des[1] = cmd_des(StateIdx::angle_p); //pBody_RPY_des[1]*0.9+0.1*seResult.rpy[1]/2.0;//
    data_.pBody_RPY_des[2] = cmd_des(StateIdx::angle_y);

    data_.vBody_Ori_des[0] = cmd_des(StateIdx::rate_r);
    data_.vBody_Ori_des[1] = cmd_des(StateIdx::rate_p);
    data_.vBody_Ori_des[2] = cmd_des(StateIdx::rate_y);

    //contact_state = gait_skd->getContactState();
    data_.contact_state = gait_skd->getContactState();
    // END of WBC Update

    return true;
  }

  bool CMpc::Init()
  {
    return true;
  }

  void CMpc::updateMPCIfNeeded(const std::vector<int> &mpcTable, const StateCmdPtr &cmd, const est::StateEstPtr &est, const Vector3 &v_des_world)
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
    solveDenseMPC(mpcTable, est);
    //    printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

  void CMpc::solveDenseMPC(const std::vector<int> &mpcTable, const est::StateEstPtr &est)
  {
  }
}
