#include "model/quadruped_impl.h"

#include "sdrobot/leg.h"
#include "params.h"
#include "model/float_base_impl.h"
#include "dynamics/rotation.h"
#include "dynamics/inertia.h"

namespace sdrobot::model
{

  namespace
  {
    /*!
  * Flip signs of elements of a vector V depending on which leg it belongs to 一个向量V的元素的翻转符号取决于它属于哪条腿
  */
    void FlipWithSideSigns(Eigen::Ref<Vector3> ret, Eigen::Ref<Vector3 const> const &v, int const leg_id)
    {
      switch (leg_id)
      {
      case leg::idx::fr:
        ret << v[0], -v[1], v[2];
        break;
      case leg::idx::fl:
        ret << v[0], v[1], v[2];
        break;
      case leg::idx::hr:
        ret << -v[0], -v[1], v[2];
        break;
      case leg::idx::hl:
        ret << -v[0], v[1], v[2];
        break;
      default:
        throw std::runtime_error("Invalid leg id!");
      }
    }

  }

  bool QuadrupedImpl::ComputeFloatBaseModel(double g)
  {
    if (model_)
      return false;

    // rotor inertia if the rotor is oriented so it spins around the z-axis
    dynamics::RotationalInertia rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Matrix3 RY;
    dynamics::CoordinateRot(RY, dynamics::CoordinateAxis::Y, M_PI / 2);
    Matrix3 RX;
    dynamics::CoordinateRot(RX, dynamics::CoordinateAxis::X, M_PI / 2);

    dynamics::RotationalInertia rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    dynamics::RotationalInertia rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vector3 rotorCOM(0, 0, 0);

    dynamics::SpatialInertia abad_rotor_spatial_inertia, hip_rotor_spatial_inertia, knee_rotor_spatial_inertia;
    dynamics::BuildSpatialInertia(abad_rotor_spatial_inertia, 0.055, rotorCOM, rotorRotationalInertiaX);
    dynamics::BuildSpatialInertia(hip_rotor_spatial_inertia, 0.055, rotorCOM, rotorRotationalInertiaY);
    knee_rotor_spatial_inertia = hip_rotor_spatial_inertia;

    // spatial inertias
    dynamics::RotationalInertia abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vector3 abadCOM(0, 0.036, 0); // LEFT
    dynamics::SpatialInertia abad_spatial_inertia;
    dynamics::BuildSpatialInertia(abad_spatial_inertia, 0.54, abadCOM, abadRotationalInertia);

    dynamics::RotationalInertia hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vector3 hipCOM(0, 0.016, -0.02);
    dynamics::SpatialInertia hip_spatial_inertia;
    dynamics::BuildSpatialInertia(hip_spatial_inertia, 0.634, hipCOM, hipRotationalInertia);

    dynamics::RotationalInertia kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vector3 kneeCOM(0, 0, -0.061);
    dynamics::SpatialInertia knee_spatial_inertia;
    dynamics::BuildSpatialInertia(knee_spatial_inertia, 0.064, kneeCOM, kneeRotationalInertia);

    dynamics::RotationalInertia bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vector3 bodyCOM(0, 0, 0);
    dynamics::SpatialInertia body_spatial_inertia;
    dynamics::BuildSpatialInertia(body_spatial_inertia, params::model::body_mass, bodyCOM,
                                  bodyRotationalInertia);

    // locations
    Vector3 abad_location(params::model::body_length * 0.5, params::model::body_width * 0.5, 0);
    Vector3 abad_rotor_location(0.125, 0.049, 0);
    Vector3 hip_location(0, params::model::abad_link_length, 0);
    Vector3 hip_rotor_location(0, 0.04, 0);
    Vector3 knee_location(0, 0, -params::model::hip_link_length);
    Vector3 knee_rotor_location(0, 0, 0);

    auto model = std::make_shared<FloatBaseModelImpl>();
    // we assume the cheetah's body (not including rotors) can be modeled as a uniformly distributed box.
    //我们假设猎豹的身体(不包括转子)可以被建模为一个均匀分布的盒子。
    Vector3 bodyDims(params::model::body_length, params::model::body_width, params::model::body_height);

    // model->addBase(_bodyMass, Vector3(0,0,0), BuildRotationalInertia(_bodyMass,
    // bodyDims));
    model->AddBase(body_spatial_inertia);
    // add contact for the cheetah's body
    model->AddGroundContactBoxPoints(5, bodyDims);

    const int base_id = 5;
    int body_id = base_id;
    int side_sign = -1;

    Matrix3 I3 = Matrix3::Identity();

    Vector3 location, rotor_location;
    dynamics::SpatialInertia spatial_inertia, rotor_spatial_inertia;

    // loop over 4 legs
    for (int leg_id = 0; leg_id < params::model::num_leg; leg_id++)
    {
      // Ab/Ad joint
      //  int addBody(const SpatialInertia& inertia, const SpatialInertia&
      //  rotorInertia, double gearRatio,
      //              int parent, JointType jointType, CoordinateAxis jointAxis,
      //              const Matrix6& Xtree, const Matrix6& Xrot);
      body_id++;
      Matrix6 xtree_abad;
      FlipWithSideSigns(location, abad_location, leg_id);
      dynamics::BuildSpatialXform(xtree_abad, I3, location);

      Matrix6 xtree_abad_rotor;
      FlipWithSideSigns(rotor_location, abad_rotor_location, leg_id);
      dynamics::BuildSpatialXform(xtree_abad_rotor, I3, rotor_location);

      if (side_sign < 0)
      {
        dynamics::SpatialInertiaFlipAlongAxis(spatial_inertia, abad_spatial_inertia, dynamics::CoordinateAxis::Y);
        dynamics::SpatialInertiaFlipAlongAxis(rotor_spatial_inertia, abad_rotor_spatial_inertia, dynamics::CoordinateAxis::Y);
      }
      else
      {
        spatial_inertia = abad_spatial_inertia;
        rotor_spatial_inertia = abad_rotor_spatial_inertia;
      }
      model->AddBody(
          spatial_inertia,
          rotor_spatial_inertia,
          params::model::abad_gear_ratio, base_id, dynamics::JointType::Revolute,
          dynamics::CoordinateAxis::X, xtree_abad, xtree_abad_rotor);

      // Hip Joint
      body_id++;
      dynamics::RotMat RZ;
      dynamics::CoordinateRot(RZ, dynamics::CoordinateAxis::Z, M_PI);
      FlipWithSideSigns(location, hip_location, leg_id);
      FlipWithSideSigns(rotor_location, hip_rotor_location, leg_id);
      Matrix6 xtree_hip;
      dynamics::BuildSpatialXform(xtree_hip, RZ, location);
      Matrix6 xtree_hip_rotor;
      dynamics::BuildSpatialXform(xtree_hip_rotor, RZ, rotor_location);

      if (side_sign < 0)
      {
        dynamics::SpatialInertiaFlipAlongAxis(spatial_inertia, hip_spatial_inertia, dynamics::CoordinateAxis::Y);
        dynamics::SpatialInertiaFlipAlongAxis(rotor_spatial_inertia, hip_rotor_spatial_inertia, dynamics::CoordinateAxis::Y);
      }
      else
      {
        spatial_inertia = hip_spatial_inertia;
        rotor_spatial_inertia = hip_rotor_spatial_inertia;
      }

      model->AddBody(
          spatial_inertia, rotor_spatial_inertia, params::model::hip_gear_ratio,
          body_id - 1, dynamics::JointType::Revolute, dynamics::CoordinateAxis::Y, xtree_hip, xtree_hip_rotor);

      // add knee ground contact point
      model->AddGroundContactPoint(body_id, Vector3(0, 0, -params::model::hip_link_length));

      // Knee Joint
      body_id++;
      Matrix6 xtree_knee;
      dynamics::BuildSpatialXform(xtree_knee, I3, knee_location);
      Matrix6 xtree_knee_rotor;
      dynamics::BuildSpatialXform(xtree_knee_rotor, I3, knee_rotor_location);

      double signed_knee_link_y_offset = params::model::knee_link_y_offset;
      if (side_sign < 0)
      {
        dynamics::SpatialInertiaFlipAlongAxis(spatial_inertia, knee_spatial_inertia, dynamics::CoordinateAxis::Y);
        dynamics::SpatialInertiaFlipAlongAxis(rotor_spatial_inertia, knee_rotor_spatial_inertia, dynamics::CoordinateAxis::Y);
      }
      else
      {
        spatial_inertia = knee_spatial_inertia;
        rotor_spatial_inertia = knee_rotor_spatial_inertia;

        signed_knee_link_y_offset *= -1.;
      }

      model->AddBody(
          spatial_inertia,
          rotor_spatial_inertia,
          params::model::knee_gear_ratio, body_id - 1, dynamics::JointType::Revolute,
          dynamics::CoordinateAxis::Y, xtree_knee, xtree_knee_rotor);

      model->AddGroundContactPoint(
          body_id, Vector3(0, signed_knee_link_y_offset, -params::model::knee_link_length), true);

      // add foot
      //model->addGroundContactPoint(body_id, Vector3(0, 0, -_kneeLinkLength), true);

      side_sign *= -1;
    }

    model->UpdateGravity({0, 0, -g});

    model_ = std::static_pointer_cast<FloatBaseModel>(model);
    return true;
  }

  FloatBaseModel::SharedPtr const &QuadrupedImpl::GetFloatBaseModel() const
  {
    return model_;
  }

  bool QuadrupedImpl::CalcHipLocation(SdVector3f &ret, int const leg) const
  {
    double hip_location[3] = {params::model::body_length * 0.5, params::model::body_width * 0.5, 0};
    ret = {
        (leg == leg::idx::fr || leg == leg::idx::fl) ? hip_location[0] : -hip_location[0],
        (leg == leg::idx::fl || leg == leg::idx::hl) ? hip_location[1] : -hip_location[1],
        hip_location[2]};
    return true;
  }
}
