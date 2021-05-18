#pragma once

namespace sd::robot
{
  namespace leg
  {
    struct Idx
    {
      constexpr static int fr = 0; // Front Right
      constexpr static int fl = 1; // Front Left
      constexpr static int hr = 2; // Hind Right
      constexpr static int hl = 3; // Hind Left
    };

    // the i-th leg is on the left (+) or right (-) of the robot.
    // 第i条腿是在机器人的左边(+)还是右边(-)。
    constexpr int SideSigns[4] = {-1, 1, -1, 1};
  }

  struct Properties
  {
    constexpr static float body_length = 0.19 * 2;
    constexpr static float body_width = 0.049 * 2;
    constexpr static float body_height = 0.05 * 2;
    constexpr static float body_mass = 3.3;

    constexpr static float abad_gear_ratio = 6.0;
    constexpr static float hip_gear_ratio = 6.0;
    constexpr static float knee_gear_ratio = 9.33;

    constexpr static float abad_link_length = 0.062;
    constexpr static float hip_link_length = 0.209;
    constexpr static float knee_link_length = 0.195;

    constexpr static float knee_link_y_offset = 0.004;
    constexpr static float max_leg_length = 0.409;

    constexpr static float motor_kt = 0.05;
    constexpr static float motor_r = 0.173;
    constexpr static float motor_tau_max = 3.0;
    constexpr static float battery_v = 24;

    constexpr static float joint_damping = 0.01;
    constexpr static float joint_dry_friction = 0.2;
  };

} // namespace sd::robot::model
