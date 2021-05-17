#pragma once

namespace sd::robot
{
  namespace Leg
  {
    namespace Idx
    {
      constexpr int FR = 0; // Front Right
      constexpr int FL = 1; // Front Left
      constexpr int HR = 2; // Hind Right
      constexpr int HL = 3; // Hind Left
    }

    // the i-th leg is on the left (+) or right (-) of the robot.
    // 第i条腿是在机器人的左边(+)还是右边(-)。
    constexpr int SideSigns[4] = {-1, 1, -1, 1};
  }

  namespace Properties
  {
    constexpr float BodyLength = 0.19 * 2;
    constexpr float BodyWidth = 0.049 * 2;
    constexpr float BodyHeight = 0.05 * 2;
    constexpr float BodyMass = 3.3;

    constexpr float AbadGearRatio = 6.0;
    constexpr float HipGearRatio = 6.0;
    constexpr float KneeGearRatio = 9.33;

    constexpr float AbadLinkLength = 0.062;
    constexpr float HipLinkLength = 0.209;
    constexpr float KneeLinkLength = 0.195;

    constexpr float KneeLinkY_offset = 0.004;
    constexpr float MaxLegLength = 0.409;

    constexpr float MotorKT = 0.05;
    constexpr float MotorR = 0.173;
    constexpr float MotorTauMax = 3.0;
    constexpr float BatteryV = 24;

    constexpr float JointDamping = 0.01;
    constexpr float JointDryFriction = 0.2;
  } // namespace Properties

} // namespace sd::robot::model
