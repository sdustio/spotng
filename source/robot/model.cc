#include "sd/robot/model.h"
#include "sd/dynamics/rotation.h"
#include "sd/dynamics/spatial.h"
#include "sd/dynamics/inertia.h"

using namespace sd::dynamics;

namespace sd::robot
{
  Quadruped::Quadruped()
  {
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    InertiaMat rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Matrix3d RY = dynamics::CoordinateRot(dynamics::CoordinateAxis::Y, M_PI / 2);
    Matrix3d RX = dynamics::CoordinateRot(dynamics::CoordinateAxis::X, M_PI / 2);
    InertiaMat rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    InertiaMat rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vector3d rotorCOM(0, 0, 0);
    abad_rotor_spatial_inertia_ = BuildSpatialInertia(0.055, rotorCOM, rotorRotationalInertiaX);
    knee_rotor_spatial_inertia_ = hip_rotor_spatial_inertia_ = BuildSpatialInertia(0.055, rotorCOM, rotorRotationalInertiaY);

    // spatial inertias
    InertiaMat abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vector3d abadCOM(0, 0.036, 0); // LEFT
    abad_spatial_inertia_ = BuildSpatialInertia(0.54, abadCOM, abadRotationalInertia);

    Matrix3d hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vector3d hipCOM(0, 0.016, -0.02);
    hip_spatial_inertia_ = BuildSpatialInertia(0.634, hipCOM, hipRotationalInertia);

    Matrix3d kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vector3d kneeCOM(0, 0, -0.061);
    knee_spatial_inertia_ = BuildSpatialInertia(0.064, kneeCOM, kneeRotationalInertia);

    Matrix3d bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vector3d bodyCOM(0, 0, 0);
    body_spatial_inertia_ = BuildSpatialInertia(QuadrupedProperties::body_mass, bodyCOM,
                                                   bodyRotationalInertia);

    // locations
    abad_rotor_location_ = Vector3d(0.125, 0.049, 0);
    abad_location_ =
        Vector3d(QuadrupedProperties::body_length, QuadrupedProperties::body_width, 0) * 0.5;
    hip_location_ = Vector3d(0, QuadrupedProperties::abad_link_length, 0);
    hip_rotor_location_ = Vector3d(0, 0.04, 0);
    knee_location_ = Vector3d(0, 0, -QuadrupedProperties::hip_link_length);
    knee_rotor_location_ = Vector3d(0, 0, 0);
  }

  bool Quadruped::BuildModel(dynamics::FBModelPtr &model)
  {
    // we assume the cheetah's body (not including rotors) can be modeled as a uniformly distributed box.
    //我们假设猎豹的身体(不包括转子)可以被建模为一个均匀分布的盒子。
    Vector3d bodyDims(QuadrupedProperties::body_length, QuadrupedProperties::body_width, QuadrupedProperties::body_height);

    // model->addBase(_bodyMass, Vector3d(0,0,0), RotInertiaOfBox(_bodyMass,
    // bodyDims));
    model->AddBase(body_spatial_inertia_);
    // add contact for the cheetah's body
    model->AddGroundContactBoxPoints(5, bodyDims);

    const int base_id = 5;
    int body_id = base_id;
    int side_sign = -1;

    Matrix3d I3 = Matrix3d::Identity();

    // loop over 4 legs
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
      // Ab/Ad joint
      //  int addBody(const SpatialInertia& inertia, const SpatialInertia&
      //  rotorInertia, double gearRatio,
      //              int parent, JointType jointType, CoordinateAxis jointAxis,
      //              const Matrix6d& Xtree, const Matrix6d& Xrot);
      body_id++;
      Matrix6d xtree_abad = CreateSpatialXform(I3, leg::SideSign::WithLegSigns(abad_location_, leg_id));
      Matrix6d xtree_abad_rotor =
          CreateSpatialXform(I3, leg::SideSign::WithLegSigns(abad_rotor_location_, leg_id));

      if (side_sign < 0)
      {
        model->AddBody(
            SpatialInertiaFlipAlongAxis(abad_spatial_inertia_, CoordinateAxis::Y),
            SpatialInertiaFlipAlongAxis(abad_rotor_spatial_inertia_, CoordinateAxis::Y),
            QuadrupedProperties::abad_gear_ratio, base_id, JointType::Revolute,
            CoordinateAxis::X, xtree_abad, xtree_abad_rotor);
      }
      else
      {
        model->AddBody(
            abad_spatial_inertia_, abad_rotor_spatial_inertia_, QuadrupedProperties::abad_gear_ratio,
            base_id, JointType::Revolute, CoordinateAxis::X, xtree_abad, xtree_abad_rotor);
      }

      // Hip Joint
      body_id++;
      Matrix6d xtree_hip =
          CreateSpatialXform(CoordinateRot(CoordinateAxis::Z, M_PI),
                       leg::SideSign::WithLegSigns(hip_location_, leg_id));
      Matrix6d xtree_hip_rotor =
          CreateSpatialXform(CoordinateRot(CoordinateAxis::Z, M_PI),
                       leg::SideSign::WithLegSigns(hip_rotor_location_, leg_id));

      if (side_sign < 0)
      {
        model->AddBody(
            SpatialInertiaFlipAlongAxis(hip_spatial_inertia_, CoordinateAxis::Y),
            SpatialInertiaFlipAlongAxis(hip_rotor_spatial_inertia_, CoordinateAxis::Y),
            QuadrupedProperties::hip_gear_ratio, body_id - 1, JointType::Revolute,
            CoordinateAxis::Y, xtree_hip, xtree_hip_rotor);
      }
      else
      {

        model->AddBody(
            hip_spatial_inertia_, hip_rotor_spatial_inertia_, QuadrupedProperties::hip_gear_ratio,
            body_id - 1, JointType::Revolute, CoordinateAxis::Y, xtree_hip, xtree_hip_rotor);
      }

      // add knee ground contact point
      model->AddGroundContactPoint(body_id, Vector3d(0, 0, -QuadrupedProperties::hip_link_length));

      // Knee Joint
      body_id++;
      Matrix6d xtree_knee = CreateSpatialXform(I3, knee_location_);
      Matrix6d xtree_knee_rotor = CreateSpatialXform(I3, knee_rotor_location_);
      if (side_sign < 0)
      {
        model->AddBody(
            SpatialInertiaFlipAlongAxis(knee_spatial_inertia_, CoordinateAxis::Y),
            SpatialInertiaFlipAlongAxis(knee_rotor_spatial_inertia_, CoordinateAxis::Y),
            QuadrupedProperties::knee_gear_ratio, body_id - 1, JointType::Revolute,
            CoordinateAxis::Y, xtree_knee, xtree_knee_rotor);

        model->AddGroundContactPoint(
            body_id, Vector3d(0, QuadrupedProperties::knee_link_y_offset, -QuadrupedProperties::knee_link_length), true);
      }
      else
      {

        model->AddBody(
            knee_spatial_inertia_, knee_rotor_spatial_inertia_, QuadrupedProperties::knee_gear_ratio,
            body_id - 1, JointType::Revolute, CoordinateAxis::Y, xtree_knee, xtree_knee_rotor);

        model->AddGroundContactPoint(
            body_id, Vector3d(0, -QuadrupedProperties::knee_link_y_offset, -QuadrupedProperties::knee_link_length), true);
      }

      // add foot
      //model->addGroundContactPoint(body_id, Vector3d(0, 0, -_kneeLinkLength), true);

      side_sign *= -1;
    }

    Vector3d g(0, 0, -9.81);
    model->SetGravity(g);

    return true;
  }
}
