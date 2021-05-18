#include "robot/model.h"

namespace sd::robot
{
  using dynamics::SpatialInertia;
  using dynamics::InertiaMat;
  using dynamics::BuildSpatialInertia;

  template <typename T>
  Quadruped<T>::Quadruped()
  {
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    InertiaMat<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = dynamics::CoordinateRot<T>(dynamics::CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = dynamics::CoordinateRot<T>(dynamics::CoordinateAxis::X, M_PI / 2);
    InertiaMat<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    InertiaMat<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vec3<T> rotorCOM(0, 0, 0);
    abad_rotor_spatial_inertia_ = BuildSpatialInertia<T>(0.055, rotorCOM, rotorRotationalInertiaX);
    knee_rotor_spatial_inertia_ = hip_rotor_spatial_inertia_ = BuildSpatialInertia<T>(0.055, rotorCOM, rotorRotationalInertiaY);

    // spatial inertias
    InertiaMat<T> abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0, 0.036, 0); // LEFT
    abad_spatial_inertia_ = BuildSpatialInertia<T>(0.54, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(0, 0.016, -0.02);
    hip_spatial_inertia_ = BuildSpatialInertia<T>(0.634, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0, 0, -0.061);
    knee_spatial_inertia_ = BuildSpatialInertia<T>(0.064, kneeCOM, kneeRotationalInertia);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(0, 0, 0);
    body_spatial_inertia_ = BuildSpatialInertia<T>(Properties::body_mass, bodyCOM,
                                              bodyRotationalInertia);

    // locations
    abad_rotor_location_ = Vec3<T>(0.125, 0.049, 0);
    abad_location_ =
        Vec3<T>(Properties::body_length, Properties::body_width, 0) * 0.5;
    hip_location_ = Vec3<T>(0, Properties::abad_link_length, 0);
    hip_rotor_location_ = Vec3<T>(0, 0.04, 0);
    knee_location_ = Vec3<T>(0, 0, -Properties::hip_link_length);
    knee_rotor_location_ = Vec3<T>(0, 0, 0);
  }

  template class Quadruped<double>;
  template class Quadruped<float>;
}
