#include "sdrobot/controllers/wbc.h"
#include "sdrobot/dynamics/rotation.h"
#include "sdrobot/robot/model.h"

#include "controllers/wbc/task/body_pos.h"
#include "controllers/wbc/task/body_ori.h"
#include "controllers/wbc/task/link_pos.h"
#include "controllers/wbc/contact/single.h"

namespace sdrobot::ctrl
{
  Wbc::Wbc(
      const dynamics::FBModelPtr &model,
      double weight) : model_(model),
                       _full_config(robot::ModelAttrs::num_act_joint),
                       _tau_ff(robot::ModelAttrs::num_act_joint),
                       _des_jpos(robot::ModelAttrs::num_act_joint),
                       _des_jvel(robot::ModelAttrs::num_act_joint),
                       _Kp_joint(robot::DynamicsAttrs::kp_joint),
                       _Kd_joint(robot::DynamicsAttrs::kd_joint),
                       _body_pos_task(std::make_shared<wbc::TaskBodyPos>(model)),
                       _body_ori_task(std::make_shared<wbc::TaskBodyOri>(model)),
                       _foot_task({
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::fr),
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::fl),
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::hr),
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::hl),
                       }),
                       _foot_contact({
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::fr),
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::fl),
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::hr),
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::hl),
                       }),
                       _kin_wbc(robot::ModelAttrs::dim_config),
                       _wbic(robot::ModelAttrs::dim_config, weight)
  {
    _full_config.setZero();
  }

  void Wbc::Run(const WbcData &input, const StateCmdPtr &state_cmd, const est::StateEstPtr &est, LegPtr &cleg)
  {

    // Update Model
    _UpdateModel(est->GetData(), cleg->GetDatas());

    // Task & Contact Update
    _ContactTaskUpdate(input);

    // WBC Computation
    _ComputeWBC();
    _UpdateLegCMD(cleg, state_cmd);
  }

  void Wbc::_UpdateModel(const est::StateData &estdata, const robot::leg::Datas &legdata)
  {
    dynamics::FBModelState _state;
    _state.body_orientation = estdata.orientation;
    _state.body_position = estdata.position;

    for (int i = 0; i < robot::ModelAttrs::num_leg_joint; i++)
    {
      _state.body_velocity[i] = estdata.omega_body[i];
      _state.body_velocity[i + 3] = estdata.v_body[i];
      for (int leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
      {
        _state.q[3 * leg + i] = legdata[leg].q[i];
        _state.qd[3 * leg + i] = legdata[leg].qd[i];

        _full_config[3 * leg + i] = _state.q[3 * leg + i];
      }
    }
    model_->SetState(_state);
    model_->ContactJacobians();
    _grav = model_->GeneralizedGravityForce();
    _coriolis = model_->GeneralizedCoriolisForce();
    _A = model_->GetMassMatrix();
    _Ainv = _A.inverse();
  }

  void Wbc::_ComputeWBC()
  {
    // TEST
    _kin_wbc.FindConfiguration(_full_config, _task_list, _contact_list,
                                _des_jpos, _des_jvel);

    // WBIC
    _wbic.UpdateSetting(_A, _Ainv, _coriolis, _grav);
    _wbic.MakeTorque(_tau_ff, _task_list, _contact_list);
  }

  void Wbc::_UpdateLegCMD(LegPtr &cleg, const StateCmdPtr &state_cmd)
  {
    auto &cmds = cleg->GetCmdsForUpdate();

    for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      cmds[leg].Zero();
      for (int jidx(0); jidx < robot::ModelAttrs::num_leg_joint; ++jidx)
      {
        cmds[leg].tau_feed_forward[jidx] = _tau_ff[robot::ModelAttrs::num_leg_joint * leg + jidx];
        cmds[leg].q_des[jidx] = _des_jpos[robot::ModelAttrs::num_leg_joint * leg + jidx];
        cmds[leg].qd_des[jidx] = _des_jvel[robot::ModelAttrs::num_leg_joint * leg + jidx];

        cmds[leg].kp_joint(jidx, jidx) = _Kp_joint[jidx];
        cmds[leg].kd_joint(jidx, jidx) = _Kd_joint[jidx];

        auto gait = state_cmd->GetGait();

        if (gait== Gait::Bound || gait == Gait::Pronk)
        {
          cmds[leg].tau_feed_forward[jidx] *= 0.8;
        }

      }
    }

    // Knee joint non flip barrier
    for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      if (cmds[leg].q_des[2] < 0.3)
      {
        cmds[leg].q_des[2] = 0.3;
      }
      if (cleg->GetDatas()[leg].q[2] < 0.3)
      {
        double knee_pos = cleg->GetDatas()[leg].q[2];
        cmds[leg].tau_feed_forward[2] = 1. / (knee_pos * knee_pos + 0.02);
      }
    }
  }

  void Wbc::_ContactTaskUpdate(const WbcData &input)
  {
    // Wash out the previous setup
    _CleanUp();

    _body_ori_task->UpdateTask(
        input.pBody_RPY_des,
        input.vBody_Ori_des,
        Vector3::Zero());
    _body_pos_task->UpdateTask(
        input.pBody_des,
        input.vBody_des,
        input.aBody_des);

    _task_list.push_back(_body_ori_task);
    _task_list.push_back(_body_pos_task);

    for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      if (input.contact_state[leg] > 0.)
      { // Contact
        _foot_contact[leg]->setRFDesired(input.Fr_des[leg]);
        _foot_contact[leg]->UpdateContact();
        _contact_list.push_back(_foot_contact[leg]);
      }
      else
      { // No Contact (swing)
        _foot_task[leg]->UpdateTask(
            input.pFoot_des[leg],
            input.vFoot_des[leg],
            input.aFoot_des[leg]);
        //zero_vec3);
        _task_list.push_back(_foot_task[leg]);
      }
    }
  }

  void Wbc::_CleanUp()
  {
    _contact_list.clear();
    _task_list.clear();
  }
}
