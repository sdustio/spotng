#pragma once

#include <memory>
#include <array>

#include "sd/estimators/state_est.h"
#include "sd/dynamics/fb_model.h"
#include "sd/controllers/leg.h"

namespace sd::ctrl
{
  struct WbcData
  {
    Vector3d pBody_des;
    Vector3d vBody_des;
    Vector3d aBody_des;
    Vector3d pBody_RPY_des;
    Vector3d vBody_Ori_des;

    std::array<Vector3d, 4> pFoot_des;
    std::array<Vector3d, 4> vFoot_des;
    std::array<Vector3d, 4> aFoot_des;
    std::array<Vector3d, 4> Fr_des;

    Vector4d contact_state;
  };

  class Wbc
  {
  public:
    Wbc(const dynamics::FBModelPtr &model, double weight);
    void Run(const WbcData &, const est::StateEstPtr &, LegPtr &);

  private:
    void _UpdateModel(const est::StateData &, const robot::leg::Datas &);
    void _ComputeWBC();
    void _UpdateLegCMD(LegPtr &);
    void _ContactTaskUpdate(const WbcData &, LegPtr &);

    dynamics::FBModelPtr model_;
    VectorXd _full_config;

    // TODO rename vars
    MatrixXd _A;
    MatrixXd _Ainv;
    VectorXd _grav;
    VectorXd _coriolis;
  };

  using WbcPtr = std::shared_ptr<Wbc>;
}
