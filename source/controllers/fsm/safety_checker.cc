#include "sdrobot/controllers/fsm.h"

namespace sdrobot::ctrl::fsm
{
  bool SafetyChecker::CheckSafeOrientation(const est::StateData &est)
  {
    return (abs(est.rpy(0)) < 0.5 && abs(est.rpy(1)) < 0.5);
  }

  bool SafetyChecker::CheckPDesFoot(LegPtr &cleg)
  {
    // Assumed safe to start
    bool check_safe = true;

    // Safety parameters
    double max_angle = 1.0472; // 60 degrees (should be changed)
    double max_p_des = robot::ModelAttrs::max_leg_length * sin(max_angle);

    robot::leg::Cmds &cmds = cleg->GetCmdsForUpdate();
    // Check all of the legs
    for (size_t leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
    {
      // Keep the foot from going too far from the body in +x
      if (cmds[leg].p_des(0) > max_p_des)
      {
        cmds[leg].p_des(0) = max_p_des;
        check_safe = false;
      }

      // Keep the foot from going too far from the body in -x
      if (cmds[leg].p_des(0) < -max_p_des)
      {
        cmds[leg].p_des(0) = -max_p_des;
        check_safe = false;
      }

      // Keep the foot from going too far from the body in +y
      if (cmds[leg].p_des(1) > max_p_des)
      {
        cmds[leg].p_des(1) = max_p_des;
        check_safe = false;
      }

      // Keep the foot from going too far from the body in -y
      if (cmds[leg].p_des(1) < -max_p_des)
      {
        cmds[leg].p_des(1) = -max_p_des;
        check_safe = false;
      }

      // Keep the leg under the motor module (don't raise above body or crash into
      // module)
      if (cmds[leg].p_des(2) >
          -robot::ModelAttrs::max_leg_length / 4)
      {
        cmds[leg].p_des(2) =
            -robot::ModelAttrs::max_leg_length / 4;
        check_safe = false;
      }

      // Keep the foot within the kinematic limits
      if (cmds[leg].p_des(2) <
          -robot::ModelAttrs::max_leg_length)
      {
        cmds[leg].p_des(2) =
            -robot::ModelAttrs::max_leg_length;
        check_safe = false;
      }
    }

    // Return true if all desired positions are safe 如果所有需要的位置都是安全的，则返回true
    return check_safe;
  }

  bool SafetyChecker::CheckForceFeedForward(LegPtr &cleg)
  {
    // Assumed safe to start
    bool check_safe = true;

    // Initialize maximum vertical and lateral forces
    double max_lateral_force = 350.;
    double max_vertical_force = 350.;

    robot::leg::Cmds &cmds = cleg->GetCmdsForUpdate();

    // Check all of the legs
    for (size_t leg = 0; leg < robot::ModelAttrs::num_leg; leg++)
    {
      // Limit the lateral forces in +x body frame
      if (cmds[leg].force_feed_forward(0) >
          max_lateral_force)
      {
        cmds[leg].force_feed_forward(0) = max_lateral_force;
        check_safe = false;
      }

      // Limit the lateral forces in -x body frame
      if (cmds[leg].force_feed_forward(0) <
          -max_lateral_force)
      {
        cmds[leg].force_feed_forward(0) =
            -max_lateral_force;
        check_safe = false;
      }

      // Limit the lateral forces in +y body frame
      if (cmds[leg].force_feed_forward(1) >
          max_lateral_force)
      {
        cmds[leg].force_feed_forward(1) = max_lateral_force;
        check_safe = false;
      }

      // Limit the lateral forces in -y body frame
      if (cmds[leg].force_feed_forward(1) <
          -max_lateral_force)
      {
        cmds[leg].force_feed_forward(1) =
            -max_lateral_force;
        check_safe = false;
      }

      // Limit the vertical forces in +z body frame
      if (cmds[leg].force_feed_forward(2) >
          max_vertical_force)
      {
        cmds[leg].force_feed_forward(2) =
            max_vertical_force;
        check_safe = false;
      }

      // Limit the vertical forces in -z body frame
      if (cmds[leg].force_feed_forward(2) <
          -max_vertical_force)
      {
        cmds[leg].force_feed_forward(2) =
            -max_vertical_force;
        check_safe = false;
      }
    }

    // Return true if all feed forward forces are safe
    return check_safe;
  }
}
