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
    Vector3d pBody_des;
    Vector3d vBody_des;
    Vector3d aBody_des;

    Vector3d pBody_RPY_des;
    Vector3d vBody_Ori_des;

    std::array<Vector3d, 4> pFoot_des;
    std::array<Vector3d, 4> vFoot_des;
    std::array<Vector3d, 4> aFoot_des;

    Vector3d Fr_des[4];

    Vector4d contact_state;
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
