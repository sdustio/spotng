#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  bool SafetyChecker::PreCheck(const StateCtrlPtr &ctrl, robot::Mode mode)
  {
    if (ctrl->NeedCheckSafeOrientation() && mode != robot::Mode::RecoveryStand)
    {
      return CheckSafeOrientation();
    }
    return true;
  }

  bool SafetyChecker::PostCheck(const StateCtrlPtr &ctrl, robot::Mode mode) {

  }

  bool SafetyChecker::CheckSafeOrientation() {}

  bool SafetyChecker::CheckPDesFoot() {}

  bool SafetyChecker::CheckForceFeedForward() {}
}
