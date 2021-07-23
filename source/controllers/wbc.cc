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
                       full_config_(robot::ModelAttrs::num_act_joint),
                       tau_ff_(robot::ModelAttrs::num_act_joint),
                       des_jpos_(robot::ModelAttrs::num_act_joint),
                       des_jvel_(robot::ModelAttrs::num_act_joint),
                       Kp_joint_(robot::DynamicsAttrs::kp_joint),
                       Kd_joint_(robot::DynamicsAttrs::kd_joint),
                       body_pos_task_(std::make_shared<wbc::TaskBodyPos>(model)),
                       body_ori_task_(std::make_shared<wbc::TaskBodyOri>(model)),
                       foot_task_({
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::fr),
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::fl),
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::hr),
                           std::make_shared<wbc::TaskLinkPos>(model, robot::LinkId::hl),
                       }),
                       foot_contact_({
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::fr),
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::fl),
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::hr),
                           std::make_shared<wbc::ContactSingle>(model, robot::LinkId::hl),
                       }),
                       kin_wbc_(robot::ModelAttrs::dim_config),
                       wbic_(robot::ModelAttrs::dim_config, weight)
  {
    full_config_.setZero();
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

        full_config_[3 * leg + i] = _state.q[3 * leg + i];
      }
    }
    model_->SetState(_state);
    model_->ContactJacobians();
    grav_ = model_->GeneralizedGravityForce();
    coriolis_ = model_->GeneralizedCoriolisForce();
    A_ = model_->GetMassMatrix();
    Ainv_ = A_.inverse();
  }

  void Wbc::_ComputeWBC()
  {
    // TEST
    kin_wbc_.FindConfiguration(full_config_, task_list_, contact_list_,
                                des_jpos_, des_jvel_);

    // WBIC
    wbic_.UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbic_.MakeTorque(tau_ff_, task_list_, contact_list_);
  }

  void Wbc::_UpdateLegCMD(LegPtr &cleg, const StateCmdPtr &state_cmd)
  {
    auto &cmds = cleg->GetCmdsForUpdate();

    for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      cmds[leg].Zero();
      for (int jidx(0); jidx < robot::ModelAttrs::num_leg_joint; ++jidx)
      {
        cmds[leg].tau_feed_forward[jidx] = tau_ff_[robot::ModelAttrs::num_leg_joint * leg + jidx];
        cmds[leg].q_des[jidx] = des_jpos_[robot::ModelAttrs::num_leg_joint * leg + jidx];
        cmds[leg].qd_des[jidx] = des_jvel_[robot::ModelAttrs::num_leg_joint * leg + jidx];

        cmds[leg].kp_joint(jidx, jidx) = Kp_joint_[jidx];
        cmds[leg].kd_joint(jidx, jidx) = Kd_joint_[jidx];

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

    body_ori_task_->UpdateTask(
        input.p_body_rpy_des,
        input.vbody_ori_des,
        Vector3::Zero());
    body_pos_task_->UpdateTask(
        input.p_body_des,
        input.v_body_des,
        input.a_body_des);

    task_list_.push_back(body_ori_task_);
    task_list_.push_back(body_pos_task_);

    for (int leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      if (input.contact_state[leg] > 0.)
      { // Contact
        foot_contact_[leg]->SetRFDesired(input.Fr_des[leg]);
        foot_contact_[leg]->UpdateContact();
        contact_list_.push_back(foot_contact_[leg]);
      }
      else
      { // No Contact (swing)
        foot_task_[leg]->UpdateTask(
            input.p_foot_des[leg],
            input.v_foot_des[leg],
            input.a_foot_des[leg]);
        //zero_vec3);
        task_list_.push_back(foot_task_[leg]);
      }
    }
  }

  void Wbc::_CleanUp()
  {
    contact_list_.clear();
    task_list_.clear();
  }
}
