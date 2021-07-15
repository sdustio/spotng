#pragma once

#include <memory>
#include <array>

#include "sdrobot/types.h"
#include "sdrobot/estimators/state_est.h"
#include "sdrobot/controllers/leg.h"
#include "sdrobot/controllers/state_cmd.h"


namespace sdrobot::ctrl
{

  struct MpcData
  {
    Vector3 pBody_des;
    Vector3 vBody_des;
    Vector3 aBody_des;

    Vector3 pBody_RPY_des;
    Vector3 vBody_Ori_des;

    std::array<Vector3, 4> pFoot_des;
    std::array<Vector3, 4> vFoot_des;
    std::array<Vector3, 4> aFoot_des;

    Vector3 Fr_des[4];

    Vector4 contact_state;
  };

  class Mpc
  {
  public:
    virtual bool Init() = 0;
    virtual bool Run(LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est) = 0;
    virtual const MpcData &GetData() = 0;
  };

  using MpcPtr = std::shared_ptr<Mpc>;
}
