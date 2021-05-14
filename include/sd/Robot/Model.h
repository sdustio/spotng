#pragma once

namespace sd::robot::model
{
  namespace LegIdx {
    constexpr int FR = 0; // Front Right
    constexpr int FL = 1; // Front Left
    constexpr int HR = 2; // Hind Right
    constexpr int HL = 3; // Hind Left
  }

  class Quadruped {
    private:
      static const float mBodyLength;
      static const float mBodyWidth;
      static const float mBodyHeight;
      static const float mBodyMass;

      static const float mAbadGearRatio;
      static const float mHipGearRatio;
      static const float mKneeGearRatio;

      static const float mAbadLinkLength;
      static const float mHipLinkLength;
      static const float mKneeLinkLength;

      static const float mKneeLinkY_offset;
      static const float mMaxLegLength;

      static const float mMotorKT;
      static const float mMotorR;
      static const float mMotorTauMax;
      static const float mBatteryV;

      static const float mJointDamping;
      static const float mJointDryFriction;

  };

} // namespace sd::robot::model
