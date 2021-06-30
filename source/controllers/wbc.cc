#include "sd/controllers/wbc.h"

namespace sd::ctrl
{
  Wbc::Wbc(const dynamics::FBModelPtr &model, [[maybe_unused]] double weight) : model_(model)
  {
  }

  void Wbc::Run([[maybe_unused]] const WbcData& data, [[maybe_unused]] const est::StateEstPtr &est, [[maybe_unused]] LegPtr &cleg)
  {
  }
}
