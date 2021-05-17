#include "Robot/Model.hpp"

namespace sd::robot::model
{
  using sd::dynamics::BuildSpatialInertia;
  using sd::dynamics::InertiaMat;

  template <typename T>
  Quadruped<T>::Quadruped()
  {
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    InertiaMat<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = sd::dynamics::CoordinateRot<T>(sd::dynamics::CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = sd::dynamics::CoordinateRot<T>(sd::dynamics::CoordinateAxis::X, M_PI / 2);
    InertiaMat<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    InertiaMat<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vec3<T> rotorCOM(0, 0, 0);
    mAbadRotorSpatialInertia = BuildSpatialInertia<T>(0.055, rotorCOM, rotorRotationalInertiaX);
    mKneeRotorSpatialInertia = mHipRotorSpatialInertia = BuildSpatialInertia<T>(0.055, rotorCOM, rotorRotationalInertiaY);

    // spatial inertias
    InertiaMat<T> abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0, 0.036, 0); // LEFT
    mAbadSpatialInertia = BuildSpatialInertia<T>(0.54, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(0, 0.016, -0.02);
    mHipSpatialInertia = BuildSpatialInertia<T>(0.634, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0, 0, -0.061);
    mKneeSpatialInertia = BuildSpatialInertia<T>(0.064, kneeCOM, kneeRotationalInertia);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(0, 0, 0);
    mBodySpatialInertia = BuildSpatialInertia<T>(Properties::BodyMass, bodyCOM,
                                              bodyRotationalInertia);

    // locations
    mAbadRotorLocation = Vec3<T>(0.125, 0.049, 0);
    mAbadLocation =
        Vec3<T>(Properties::BodyLength, Properties::BodyWidth, 0) * 0.5;
    mHipLocation = Vec3<T>(0, Properties::AbadLinkLength, 0);
    mHipRotorLocation = Vec3<T>(0, 0.04, 0);
    mKneeLocation = Vec3<T>(0, 0, -Properties::HipLinkLength);
    mKneeRotorLocation = Vec3<T>(0, 0, 0);
  }

  template class Quadruped<double>;
  template class Quadruped<float>;
}
