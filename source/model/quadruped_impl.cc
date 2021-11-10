#include "model/quadruped_impl.h"

#include <memory>

#include "dynamics/inertia.h"
#include "dynamics/rotation.h"
#include "model/float_base_impl.h"
#include "sdquadx/consts.h"
#include "sdquadx/leg.h"

namespace sdquadx::model {

namespace {
/*!
 * Flip signs of elements of a vector V depending on which leg it belongs to
 * 一个向量V的元素的翻转符号取决于它属于哪条腿
 */
bool FlipWithSideSigns(Eigen::Ref<Vector3> ret, Eigen::Ref<Vector3 const> const &v, int const leg_id) {
  switch (leg_id) {
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
      return false;
  }
  return true;
}

}  // namespace

bool QuadrupedImpl::ComputeFloatBaseModel() {
  if (model_) return false;

  // locations
  Matrix3 RY;
  dynamics::CoordinateRot(RY, dynamics::CoordinateAxis::Y, consts::math::kPI / 2);
  Matrix3 RX;
  dynamics::CoordinateRot(RX, dynamics::CoordinateAxis::X, consts::math::kPI / 2);

  // spatial inertias
  Eigen::Map<dynamics::InertiaTensor const> abadInertiaTensor(opts_->model.inertia_abad.data());
  Eigen::Map<Vector3 const> abadCOM(opts_->model.com_abad_fl.data());  // LEFT
  dynamics::SpatialInertia abad_spatial_inertia;
  dynamics::BuildSpatialInertia(abad_spatial_inertia, opts_->model.mass_abad, abadCOM, abadInertiaTensor);

  Eigen::Map<dynamics::InertiaTensor const> hipInertiaTensor(opts_->model.inertia_hip.data());
  Eigen::Map<Vector3 const> hipCOM(opts_->model.com_hip_fl.data());  // LEFT
  dynamics::SpatialInertia hip_spatial_inertia;
  dynamics::BuildSpatialInertia(hip_spatial_inertia, opts_->model.mass_hip, hipCOM, hipInertiaTensor);

  Eigen::Map<dynamics::InertiaTensor const> kneeInertiaTensorRotated(opts_->model.inertia_knee.data());
  dynamics::InertiaTensor kneeInertiaTensor;
  kneeInertiaTensor = RY * kneeInertiaTensorRotated * RY.transpose();
  Eigen::Map<Vector3 const> kneeCOM(opts_->model.com_knee_fl.data());
  dynamics::SpatialInertia knee_spatial_inertia;
  dynamics::BuildSpatialInertia(knee_spatial_inertia, opts_->model.mass_knee, kneeCOM, kneeInertiaTensor);

  Eigen::Map<dynamics::InertiaTensor const> bodyInertiaTensor(opts_->model.inertia_body.data());
  Eigen::Map<Vector3 const> bodyCOM(opts_->model.com_body.data());
  dynamics::SpatialInertia body_spatial_inertia;
  dynamics::BuildSpatialInertia(body_spatial_inertia, opts_->model.mass_body, bodyCOM, bodyInertiaTensor);

  auto model = std::make_shared<FloatBaseModelImpl>();
  // we assume the cheetah's body (not including rotors) can be modeled as a
  // uniformly distributed box.
  // 我们假设猎豹的身体(不包括转子)可以被建模为一个均匀分布的盒子。
  Vector3 bodyDims(opts_->model.body_length, opts_->model.body_width, opts_->model.body_height);

  // model->addBase(_bodyMass, Vector3(0,0,0), BuildInertiaTensor(_bodyMass,
  // bodyDims));
  model->AddBase(body_spatial_inertia);
  // add contact for the cheetah's body
  model->AddGroundContactBoxPoints(5, bodyDims);

  const int base_id = 5;
  int body_id = base_id;
  int side_sign = -1;

  Matrix3 I3 = Matrix3::Identity();

  Vector3 location;
  dynamics::SpatialInertia spatial_inertia;

  // loop over 4 legs
  for (int leg_id = 0; leg_id < consts::model::kNumLeg; leg_id++) {
    // Ab/Ad joint
    //  int addBody(const SpatialInertia& inertia, const SpatialInertia&
    //  rotorInertia, fptype gearRatio,
    //              int parent, JointType jointType, CoordinateAxis jointAxis,
    //              const Matrix6& Xtree, const Matrix6& Xrot);
    body_id++;
    Matrix6 xtree_abad;
    FlipWithSideSigns(location, ToConstEigenTp(opts_->model.location_abad_fl), leg_id);
    dynamics::BuildSpatialXform(xtree_abad, I3, location);

    if (side_sign < 0) {
      dynamics::SpatialInertiaFlipAlongAxis(spatial_inertia, abad_spatial_inertia, dynamics::CoordinateAxis::Y);
    } else {
      spatial_inertia = abad_spatial_inertia;
    }
    model->AddBody(spatial_inertia, rotor_spatial_inertia, opts_->model.gear_ratio_abad, base_id,
                   dynamics::JointType::Revolute, dynamics::CoordinateAxis::X, xtree_abad, xtree_abad_rotor);

    // Hip Joint
    body_id++;
    dynamics::RotMat RZ;
    dynamics::CoordinateRot(RZ, dynamics::CoordinateAxis::Z, consts::math::kPI);
    FlipWithSideSigns(location, ToConstEigenTp(opts_->model.location_hip_fl), leg_id);
    FlipWithSideSigns(rotor_location, ToConstEigenTp(opts_->model.location_hip_rotor_fl), leg_id);
    Matrix6 xtree_hip;
    dynamics::BuildSpatialXform(xtree_hip, RZ, location);
    Matrix6 xtree_hip_rotor;
    dynamics::BuildSpatialXform(xtree_hip_rotor, RZ, rotor_location);

    if (side_sign < 0) {
      dynamics::SpatialInertiaFlipAlongAxis(spatial_inertia, hip_spatial_inertia, dynamics::CoordinateAxis::Y);
      dynamics::SpatialInertiaFlipAlongAxis(rotor_spatial_inertia, hip_rotor_spatial_inertia,
                                            dynamics::CoordinateAxis::Y);
    } else {
      spatial_inertia = hip_spatial_inertia;
      rotor_spatial_inertia = hip_rotor_spatial_inertia;
    }

    model->AddBody(spatial_inertia, rotor_spatial_inertia, opts_->model.gear_ratio_hip, body_id - 1,
                   dynamics::JointType::Revolute, dynamics::CoordinateAxis::Y, xtree_hip, xtree_hip_rotor);

    // add knee ground contact point
    model->AddGroundContactPoint(body_id, Vector3(0, 0, -opts_->model.link_length_hip));

    // Knee Joint
    body_id++;
    Matrix6 xtree_knee;
    dynamics::BuildSpatialXform(xtree_knee, I3, ToConstEigenTp(opts_->model.location_knee_fl));
    Matrix6 xtree_knee_rotor;
    dynamics::BuildSpatialXform(xtree_knee_rotor, I3, ToConstEigenTp(opts_->model.location_knee_rotor_fl));

    fpt_t signed_knee_link_y_offset = opts_->model.link_yoffset_knee;
    if (side_sign < 0) {
      dynamics::SpatialInertiaFlipAlongAxis(spatial_inertia, knee_spatial_inertia, dynamics::CoordinateAxis::Y);
      dynamics::SpatialInertiaFlipAlongAxis(rotor_spatial_inertia, knee_rotor_spatial_inertia,
                                            dynamics::CoordinateAxis::Y);
    } else {
      spatial_inertia = knee_spatial_inertia;
      rotor_spatial_inertia = knee_rotor_spatial_inertia;

      signed_knee_link_y_offset *= -1.;
    }

    model->AddBody(spatial_inertia, rotor_spatial_inertia, opts_->model.gear_ratio_knee, body_id - 1,
                   dynamics::JointType::Revolute, dynamics::CoordinateAxis::Y, xtree_knee, xtree_knee_rotor);

    model->AddGroundContactPoint(body_id, Vector3(0, signed_knee_link_y_offset, -opts_->model.link_length_knee), true);

    // add foot
    // model->addGroundContactPoint(body_id, Vector3(0, 0, -_kneeLinkLength),
    // true);

    side_sign *= -1;
  }

  model->UpdateGravity({0, 0, -opts_->gravity});

  model_ = std::static_pointer_cast<FloatBaseModel>(model);
  return true;
}

FloatBaseModel::SharedPtr const &QuadrupedImpl::GetFloatBaseModel() const { return model_; }

bool QuadrupedImpl::CalcAbadLocation(SdVector3f &ret, int const leg) const {
  ret = {(leg == leg::idx::fr || leg == leg::idx::fl) ? opts_->model.location_abad_fl[0]
                                                      : -opts_->model.location_abad_fl[0],
         (leg == leg::idx::fl || leg == leg::idx::hl) ? opts_->model.location_abad_fl[1]
                                                      : -opts_->model.location_abad_fl[1],
         opts_->model.location_abad_fl[2]};
  return true;
}
}  // namespace sdquadx::model
