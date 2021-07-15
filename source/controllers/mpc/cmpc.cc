#include "controllers/mpc/cmpc.h"

namespace sdrobot::ctrl::mpc
{
  CMpc::CMpc(double _dt, int _iterations_between_mpc) : dt_(_dt),
                                                        iterationsBetweenMPC(_iterations_between_mpc)
  {
  }

  bool CMpc::Run(LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est)
  {
    bool omniMode = false;

    const auto &seResult = est->GetData();
    // TODO check! data._stateEstimator->setRemoterVelocityResult(Vector3(_x_vel_des, _y_vel_des, _yaw_turn_rate));

    /*
    // Check if transition to standing
    if (((gaitNumber == 4) && current_gait != 4) || firstRun)
    {
      stand_traj[0] = seResult.position[0];
      stand_traj[1] = seResult.position[1];
      stand_traj[2] = 0.21;
      stand_traj[3] = 0;
      stand_traj[4] = 0;
      stand_traj[5] = seResult.rpy[2];
      world_position_desired[0] = stand_traj[0];
      world_position_desired[1] = stand_traj[1];
    }

    // pick gait
    Gait *gait = &trotting;
    if (gaitNumber == 1)
      gait = &bounding;
    else if (gaitNumber == 2)
      gait = &pronking;
    else if (gaitNumber == 3)
      gait = &slowtrotting; //random; pacing;//
    else if (gaitNumber == 4)
      gait = &standing;
    else if (gaitNumber == 5)
      gait = &trotRunning; //
    else if (gaitNumber == 6)
      gait = &walking;
    else if (gaitNumber == 7)
      gait = &random2;
    else if (gaitNumber == 8)
      gait = &pacing;
    else if (gaitNumber == 101)
      gait = &trotting; //
    else if (gaitNumber == 102)
      gait = &trotting; //
    current_gait = gaitNumber;

    gait->setIterations(iterationsBetweenMPC, iterationCounter);
    jumping.setIterations(iterationsBetweenMPC, iterationCounter);

    jumping.setIterations(27 / 2, iterationCounter);

    //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
    // check jump trigger
    jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
                               data._desiredStateCommand->trigger_pressed);

    // bool too_high = seResult.position[2] > 0.29;
    // check jump action
    if (jump_state.should_jump(jumping.getCurrentGaitPhase()))
    {
      gait = &jumping;
      recompute_timing(27 / 2);
      _body_height = _body_height_jumping;
      currently_jumping = true;
    }
    else
    {
      recompute_timing(default_iterations_between_mpc);
      currently_jumping = false;
    }

    if (_body_height < 0.02)
    {
      _body_height = 0.3;
    }
    if (gaitNumber == 5)
      _body_height = 0.31;

    // integrate position setpoint
    Vector3 v_des_robot(_x_vel_des, _y_vel_des, 0);

    Vector3 v_des_world =
        omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    Vector3 v_robot = seResult.vWorld;

    //pretty_print(v_des_world, std::cout, "v des world");

    //Integral-esque pitche and roll compensation
    if (fabs(v_robot[0]) > .02) //avoid dividing by zero
    {
      rpy_int[1] += 5 * dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
    }
    if (fabs(v_robot[1]) > 0.01)
    {
      rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
    }

    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
    rpy_comp[1] = v_robot[0] * rpy_int[1];
    rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 2); //turn off for pronking

    for (int i = 0; i < 4; i++)
    {
      pFoot[i] = seResult.position +
                 seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) +
                                               data._legController->datas[i].p);
    }

    if (gait != &standing)
    {
      world_position_desired += dt * Vector3(v_des_world[0], v_des_world[1], 0);
    }

    // some first time initialization
    if (firstRun)
    {
      world_position_desired[0] = seResult.position[0];
      world_position_desired[1] = seResult.position[1];
      world_position_desired[2] = seResult.rpy[2];

      for (int i = 0; i < 4; i++)
      {

        footSwingTrajectories[i].setHeight(0.05);
        footSwingTrajectories[i].setInitialPosition(pFoot[i]);
        footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      }
      firstRun = false;
    }

    // foot placement
    for (int l = 0; l < 4; l++)
      swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

    float side_sign[4] = {-1, 1, -1, 1};
    //  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
    float interleave_y[4] = {0.0, 0.0, 0.02, -0.02};
    //float interleave_gain = -0.13;
    float interleave_gain = 0; //-0.2;
    //float v_abs = std::fabs(seResult.vBody[0]);
    float v_abs = std::fabs(v_des_robot[0]);
    for (int i = 0; i < 4; i++)
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

      footSwingTrajectories[i].setHeight(step_height); //.125);
                                                       //    Vector3 offset(0.05, side_sign[i] * .062, 0);

      Vector3 offset(0, side_sign[i] * .072, 0);
      //     Vector3 offset(0, side_sign[i] * .075, 0);

      if (i < 2)
        offset[0] = 0; //0.03;
      else
        offset[0] = -0.02; //0.02;//

      //      offset[0]=seResult.vBody[0]*0.01;

      if (gaitNumber == 101)
      {
        if (i < 2)
          offset(0) = 0.2;
        else
          offset(0) = -0.15;
        if (i % 2 == 0)
          offset(1) = -0.15;
        else
          offset(1) = 0.15;
      }
      else if (gaitNumber == 102)
      {
        if (i < 2)
          offset(0) = -0.06;
        else
          offset(0) = 0.06;
      }
      //      if(i<2) // place the front legs forward
      //          offset[0]=0.015;//0.055;

      //      if(gaitNumber==1) //bound gait
      //      {
      ////         if(i>=2)
      ////            offset[0]=-0.05;
      //
      //      }
      if (gaitNumber == 6) //walk gait
      {
        if (i == 0)
          offset(1) = -0.085 * (1 - fabs(v_des_robot[0]) / 2.0);
        else if (i == 1)
          offset(1) = 0.085 * (1 - fabs(v_des_robot[0]) / 2.0);
      }

      //      if(gaitNumber==3) //pacing
      //      {
      //          if(i%2==0)
      //             offset[1]=-0.05;
      //          else
      //              offset[1]=0.05;
      //      }

      Vector3 pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

      pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
      float stance_time = gait->getCurrentStanceTime(dtMPC, i);
      Vector3 pYawCorrected =
          coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

      Vector3 des_vel;
      des_vel[0] = _x_vel_des;
      des_vel[1] = _y_vel_des;
      des_vel[2] = 0.0;

      Vector3 Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

      //+ seResult.vWorld * swingTimeRemaining[i];

      float p_rel_max = 0.35f;
      //    float p_rel_max = 0.3f;

      // Using the estimated velocity is correct
      //Vector3 des_vel_world = seResult.rBody.transpose() * des_vel;
      float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
                      .1f * (seResult.vWorld[0] - v_des_world[0]) +
                      (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);

      if (fabs(pfx_rel) > p_rel_max)
        printf("!!!!!!!!!!!!!!!!out of the max step\n");

      float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
                      .09f * (seResult.vWorld[1] - v_des_world[1]) +
                      (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
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

    // load LCM leg swing gains
    Kp << 700, 0, 0,
        0, 700, 0,
        0, 0, 250;
    Kp_stance = 0 * Kp;

    //  Kd << 7, 0, 0,
    //     0, 7, 0,
    //     0, 0, 7;
    Kd << 14, 0, 0,
        0, 14, 0,
        0, 0, 14;
    Kd_stance = Kd;
    // gait
    Vector4 contactStates = gait->getContactState();
    //  std::cout<<"contactStates:"<<contactStates<<std::endl;
    Vector4 swingStates = gait->getSwingState();
    int *mpcTable = gait->getMpcTable();
    updateMPCIfNeeded(mpcTable, data, omniMode);

    //  StateEstimator* se = hw_i->state_estimator;
    Vector4 se_contactState(0, 0, 0, 0);


    for (int foot = 0; foot < 4; foot++)
    {
      float contactState = contactStates[foot];
      float swingState = swingStates[foot];
      if (swingState > 0) // foot is in swing
      {
        if (firstSwing[foot])
        {
          firstSwing[foot] = false;
          footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        }

        footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
        //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

        Vector3 pDesFootWorld = footSwingTrajectories[foot].getPosition();
        Vector3 vDesFootWorld = footSwingTrajectories[foot].getVelocity();
        Vector3 pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
        Vector3 vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

        // Update for WBC
        pFoot_des[foot] = pDesFootWorld;
        vFoot_des[foot] = vDesFootWorld;
        aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

        if (!data.userParameters->use_wbc)
        {
          // Update leg control command regardless of the usage of WBIC
          data._legController->commands[foot].pDes = pDesLeg;
          data._legController->commands[foot].vDes = vDesLeg;
          data._legController->commands[foot].kpCartesian = Kp;
          data._legController->commands[foot].kdCartesian = Kd;
        }
      }
      else // foot is in stance
      {
        firstSwing[foot] = true;


        Vector3 pDesFootWorld = footSwingTrajectories[foot].getPosition();
        Vector3 vDesFootWorld = footSwingTrajectories[foot].getVelocity();
        Vector3 pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
        Vector3 vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
        //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

        if (!data.userParameters->use_wbc)
        {
          data._legController->commands[foot].pDes = pDesLeg;
          data._legController->commands[foot].vDes = vDesLeg;
          data._legController->commands[foot].kpCartesian = Kp_stance;
          data._legController->commands[foot].kdCartesian = Kd_stance;

          data._legController->commands[foot].forceFeedForward = f_ff[foot];
          data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

          //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
          //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
          // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
        }
        else
        { // Stance foot damping
          data._legController->commands[foot].pDes = pDesLeg;
          data._legController->commands[foot].vDes = vDesLeg;
          data._legController->commands[foot].kpCartesian = 0. * Kp_stance;
          data._legController->commands[foot].kdCartesian = Kd_stance;
        }
        //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
        se_contactState[foot] = contactState;

        // Update for WBC
        //Fr_des[foot] = -f_ff[foot];
      }
    }

    // se->set_contact_state(se_contactState); todo removed
    data._stateEstimator->setContactPhase(se_contactState);

    // Update For WBC
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = _body_height;

    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0.;

    aBody_des.setZero();

    //  pBody_RPY_des[0] = 0.;
    //  pBody_RPY_des[1] = 0.;
    pBody_RPY_des[0] = 0;                                           //data._desiredStateCommand->data.stateDes(3); // pBody_RPY_des[0]*0.9+0.1*seResult.rpy[0]/2.0;//
    pBody_RPY_des[1] = data._desiredStateCommand->data.stateDes(4); //pBody_RPY_des[1]*0.9+0.1*seResult.rpy[1]/2.0;//
    pBody_RPY_des[2] = _yaw_des;

    vBody_Ori_des[0] = 0.;
    vBody_Ori_des[1] = 0.;
    vBody_Ori_des[2] = _yaw_turn_rate;

    //contact_state = gait->getContactState();
    contact_state = gait->getContactState();
    // END of WBC Update

    */

    return true;
  }
}
