#pragma once

#include "sdrobot/estimate.h"
#include "sdrobot/fsm.h"
#include "sdrobot/leg.h"

namespace sdrobot::fsm {
class SafetyChecker {
 public:
  // Pre checks to make sure controls are safe to run
  bool CheckSafeOrientation(
      estimate::EstimateCtrl::SharedPtr const &estctrl);  // robot's orientation is safe to control

  // Post checks to make sure controls can be sent to robot
  bool CheckPDesFoot(leg::LegCtrl::SharedPtr const &legctrl);          // desired foot position is not too far
  bool CheckForceFeedForward(leg::LegCtrl::SharedPtr const &legctrl);  // desired feedforward forces are not too large
};
}  // namespace sdrobot::fsm
