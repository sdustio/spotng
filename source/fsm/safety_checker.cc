#include <cmath>
#include "sdrobot/params.h"
#include "fsm/safety_checker.h"

namespace sdrobot::fsm
{
  bool SafetyChecker::CheckSafeOrientation(estimate::EstimateCtrl::SharedPtr const &estctrl)
  {
    auto const &state = estctrl->GetEstState();
    return (abs(state.pos_rpy[0]) < 0.5 && abs(state.pos_rpy[1]) < 0.5);
  }

  bool SafetyChecker::CheckPDesFoot(leg::LegCtrl::SharedPtr const &legctrl)
  {
    // Assumed safe to start
    bool check_safe = true;

    // Safety parameters
    auto max_p_des = params::model::kMaxLegLength * std::sin(params::interface::kMaxAngle);

    auto &cmds = legctrl->GetCmdsForUpdate();
    // Check all of the legs
    for (int leg = 0; leg < params::model::kNumLeg; leg++)
    {
      // Keep the foot from going too far from the body in +x
      if (cmds[leg].p_des[0] > max_p_des)
      {
        cmds[leg].p_des[0] = max_p_des;
        check_safe = false;
      }

      // Keep the foot from going too far from the body in -x
      if (cmds[leg].p_des[0] < -max_p_des)
      {
        cmds[leg].p_des[0] = -max_p_des;
        check_safe = false;
      }

      // Keep the foot from going too far from the body in +y
      if (cmds[leg].p_des[1] > max_p_des)
      {
        cmds[leg].p_des[1] = max_p_des;
        check_safe = false;
      }

      // Keep the foot from going too far from the body in -y
      if (cmds[leg].p_des[1] < -max_p_des)
      {
        cmds[leg].p_des[1] = -max_p_des;
        check_safe = false;
      }

      // Keep the leg under the motor module (don't raise above body or crash into
      // module)
      if (cmds[leg].p_des[2] > -params::model::kMaxLegLength / 4)
      {
        cmds[leg].p_des[2] = -params::model::kMaxLegLength / 4;
        check_safe = false;
      }

      // Keep the foot within the kinematic limits
      if (cmds[leg].p_des[2] < -params::model::kMaxLegLength)
      {
        cmds[leg].p_des[2] = -params::model::kMaxLegLength;
        check_safe = false;
      }
    }

    // Return true if all desired positions are safe 如果所有需要的位置都是安全的，则返回true
    return check_safe;
  }

  bool SafetyChecker::CheckForceFeedForward(leg::LegCtrl::SharedPtr const &legctrl)
  {
    // Assumed safe to start
    bool check_safe = true;

    auto &cmds = legctrl->GetCmdsForUpdate();

    // Check all of the legs
    for (int leg = 0; leg < params::model::kNumLeg; leg++)
    {
      // Limit the lateral forces in +x body frame
      if (cmds[leg].force_feed_forward[0] > params::interface::kMaxLateralForce)
      {
        cmds[leg].force_feed_forward[0] = params::interface::kMaxLateralForce;
        check_safe = false;
      }

      // Limit the lateral forces in -x body frame
      if (cmds[leg].force_feed_forward[0] < -params::interface::kMaxLateralForce)
      {
        cmds[leg].force_feed_forward[0] = -params::interface::kMaxLateralForce;
        check_safe = false;
      }

      // Limit the lateral forces in +y body frame
      if (cmds[leg].force_feed_forward[1] > params::interface::kMaxLateralForce)
      {
        cmds[leg].force_feed_forward[1] = params::interface::kMaxLateralForce;
        check_safe = false;
      }

      // Limit the lateral forces in -y body frame
      if (cmds[leg].force_feed_forward[1] < -params::interface::kMaxLateralForce)
      {
        cmds[leg].force_feed_forward[1] = -params::interface::kMaxLateralForce;
        check_safe = false;
      }

      // Limit the vertical forces in +z body frame
      if (cmds[leg].force_feed_forward[2] > params::interface::kMaxVerticalForce)
      {
        cmds[leg].force_feed_forward[2] = params::interface::kMaxVerticalForce;
        check_safe = false;
      }

      // Limit the vertical forces in -z body frame
      if (cmds[leg].force_feed_forward[2] < -params::interface::kMaxVerticalForce)
      {
        cmds[leg].force_feed_forward[2] = -params::interface::kMaxVerticalForce;
        check_safe = false;
      }
    }

    // Return true if all feed forward forces are safe
    return check_safe;
  }

}
