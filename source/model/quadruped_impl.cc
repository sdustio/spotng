#include "model/quadruped_impl.h"

#include "sdrobot/leg.h"
#include "params.h"
#include "dynamics/rotation.h"
#include "dynamics/inertia.h"

namespace sdrobot::model
{
  QuadrupedImpl::QuadrupedImpl()
  {
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    dynamics::InertiaMat rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Matrix3 RY;
    dynamics::CoordinateRot(RY, dynamics::CoordinateAxis::Y, M_PI / 2);
    Matrix3 RX;
    dynamics::CoordinateRot(RX, dynamics::CoordinateAxis::X, M_PI / 2);

    dynamics::InertiaMat rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    dynamics::InertiaMat rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vector3 rotorCOM(0, 0, 0);

    Eigen::Map<dynamics::SpatialInertia> abad_rotor_spatial_inertia(abad_rotor_spatial_inertia_.data());
    Eigen::Map<dynamics::SpatialInertia> hip_rotor_spatial_inertia(hip_rotor_spatial_inertia_.data());
    dynamics::BuildSpatialInertia(abad_rotor_spatial_inertia, 0.055, rotorCOM, rotorRotationalInertiaX);
    dynamics::BuildSpatialInertia(hip_rotor_spatial_inertia, 0.055, rotorCOM, rotorRotationalInertiaY);
    knee_rotor_spatial_inertia_ = hip_rotor_spatial_inertia_;

    // spatial inertias
    dynamics::InertiaMat abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vector3 abadCOM(0, 0.036, 0); // LEFT
    Eigen::Map<dynamics::SpatialInertia> abad_spatial_inertia(abad_spatial_inertia_.data());
    dynamics::BuildSpatialInertia(abad_spatial_inertia, 0.54, abadCOM, abadRotationalInertia);

    dynamics::InertiaMat hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vector3 hipCOM(0, 0.016, -0.02);
    Eigen::Map<dynamics::SpatialInertia> hip_spatial_inertia(hip_spatial_inertia_.data());
    dynamics::BuildSpatialInertia(hip_spatial_inertia, 0.634, hipCOM, hipRotationalInertia);

    dynamics::InertiaMat kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vector3 kneeCOM(0, 0, -0.061);
    Eigen::Map<dynamics::SpatialInertia> knee_spatial_inertia(knee_spatial_inertia_.data());
    dynamics::BuildSpatialInertia(knee_spatial_inertia, 0.064, kneeCOM, kneeRotationalInertia);

    dynamics::InertiaMat bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vector3 bodyCOM(0, 0, 0);
    Eigen::Map<dynamics::SpatialInertia> body_spatial_inertia(body_spatial_inertia_.data());
    dynamics::BuildSpatialInertia(body_spatial_inertia, params::model::body_mass, bodyCOM,
                                  bodyRotationalInertia);

    // locations
    abad_rotor_location_ = {0.125, 0.049, 0};
    abad_location_ = {params::model::body_length * 0.5, params::model::body_width * 0.5, 0};
    hip_location_ = {0, params::model::abad_link_length, 0};
    hip_rotor_location_ = {0, 0.04, 0};
    knee_location_ = {0, 0, -params::model::hip_link_length};
    knee_rotor_location_ = {0, 0, 0};
  }

  bool QuadrupedImpl::ComputeFloatBaseModel(double g) const
  {
    return true;
  }

  FloatBaseModel::SharedPtr const &QuadrupedImpl::GetFloatBaseModel() const
  {
    return model_;
  }

  bool QuadrupedImpl::CalcHipLocation(SdVector3f &ret, int const leg) const
  {
    ret = {
        (leg == leg::idx::fr || leg == leg::idx::fl) ? abad_location_[0] : -abad_location_[0],
        (leg == leg::idx::fl || leg == leg::idx::hl) ? abad_location_[1] : -abad_location_[1],
        abad_location_[2]};
    return true;
  }
}
