#include "sd/controllers/wbc.h"

namespace sd::ctrl
{
  Wbc::Wbc(const dynamics::FBModelPtr &model, [[maybe_unused]] double weight) : model_(model)
  {
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
  }

  void Wbc::_ComputeWBC() {}
  void Wbc::_UpdateLegCMD(LegPtr &cleg) {}
  void Wbc::_ContactTaskUpdate(const WbcData &input, LegPtr &cleg) {}
}
