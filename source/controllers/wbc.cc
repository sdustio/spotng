#include "sd/controllers/wbc.h"

namespace sd::ctrl
{
  Wbc::Wbc(const dynamics::FBModelPtr &model, [[maybe_unused]] double weight) : model_(model)
  {
    _full_config = VectorXd::Zero(robot::ModelAttrs::num_act_joint);
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
    dynamics::FBModelState _state{
        estdata.orientation,
        estdata.position,
    };
    for (size_t i = 0; i < robot::ModelAttrs::num_leg_joint; i++)
    {
      _state.body_velocity[i] = estdata.omega_body[i];
      _state.body_velocity[i + 3] = estdata.v_body[i];
      for (size_t leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
      {
        _state.q[3 * leg + i] = legdata[leg].q[i];
        _state.qd[3 * leg + i] = legdata[leg].qd[i];

        _full_config[3*leg + i] = _state.q[3*leg + i];
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
  void Wbc::_UpdateLegCMD(LegPtr &cleg) {}
  void Wbc::_ContactTaskUpdate(const WbcData &input, LegPtr &cleg) {}
}
