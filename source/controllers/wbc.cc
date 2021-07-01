#include "sd/controllers/wbc.h"

namespace sd::ctrl
{
  Wbc::Wbc(const dynamics::FBModelPtr &model, double weight) :
  model_(model),
  _full_config(robot::ModelAttrs::num_act_joint),
  _tau_ff(robot::ModelAttrs::num_act_joint),
  _des_jpos(robot::ModelAttrs::num_act_joint),
  _des_jvel(robot::ModelAttrs::num_act_joint)
  {
    //TODO _full_config size to 12
    _full_config.setZero();

    _Kp_joint.fill(5.);
    _Kd_joint.fill(1.5);
  }

  void Wbc::Run(const WbcData &input, const est::StateEstPtr &est, LegPtr &cleg)
  {

    // Update Model
    _UpdateModel(est->GetData(), cleg->GetDatas());

    // Task & Contact Update
    _ContactTaskUpdate(input, cleg);

    // WBC Computation
    _ComputeWBC();
    _UpdateLegCMD(cleg);
  }

  void Wbc::_UpdateModel(const est::StateData &estdata, const robot::leg::Datas &legdata)
  {
    dynamics::FBModelState _state;
    _state.body_orientation = estdata.orientation;
    _state.body_position = estdata.position;

    for (size_t i = 0; i < robot::ModelAttrs::num_leg_joint; i++)
    {
      _state.body_velocity[i] = estdata.omega_body[i];
      _state.body_velocity[i + 3] = estdata.v_body[i];
      for (size_t leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
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

  void Wbc::_ComputeWBC() {}
  void Wbc::_UpdateLegCMD(LegPtr &cleg)
  {
    auto &cmds = cleg->GetCmdsForUpdate();

    for (size_t leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
    {
      cmds[leg].Zero();
      for (size_t jidx(0); jidx < robot::ModelAttrs::num_leg_joint; ++jidx)
      {
        cmds[leg].tau_feed_forward[jidx] = _tau_ff[robot::ModelAttrs::num_leg_joint * leg + jidx];
        cmds[leg].q_des[jidx] = _des_jpos[robot::ModelAttrs::num_leg_joint * leg + jidx];
        cmds[leg].qd_des[jidx] = _des_jvel[robot::ModelAttrs::num_leg_joint * leg + jidx];

        cmds[leg].kp_joint(jidx, jidx) = _Kp_joint[jidx];
        cmds[leg].kd_joint(jidx, jidx) = _Kd_joint[jidx];

        /*TODO locomotion
        if (data.userParameters->cmpc_gait == 1 || data.userParameters->cmpc_gait == 2) // bound and pronk gaits
        {
          cmds[leg].tau_feed_forward[jidx] *= 0.8;
        }
        */
      }
    }

    // Knee joint non flip barrier
    for (size_t leg(0); leg < robot::ModelAttrs::num_leg; ++leg)
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
  void Wbc::_ContactTaskUpdate([[maybe_unused]] const WbcData &input, [[maybe_unused]] LegPtr &cleg) {}
}
