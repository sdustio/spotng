#include "model/float_base.h"

#include "dynamics/inertia.h"
#include "dynamics/rotation.h"

namespace sdquadx::model {
using genf_t = Eigen::Matrix<fpt_t, consts::model::kDimConfig, 1>;
using mass_t = Eigen::Matrix<fpt_t, consts::model::kDimConfig, consts::model::kDimConfig>;
using jc_t = Eigen::Matrix<fpt_t, 3, consts::model::kDimConfig>;

FBModelState &FBModel::GetStateForUpdate() {
  ResetCalculationFlags();
  return state_;
}

bool FBModel::UpdateGravity(SdVector3f const &g) {
  gravity_ = g;
  return true;
}

bool FBModel::ComputeGeneralizedMassMatrix(DynamicsData &ret) {
  CompositeInertias();
  ret.M.fill(0.);

  Eigen::Map<mass_t> M(ret.M.data());

  // Top left corner is the locked inertia of the whole system
  M.topLeftCorner<6, 6>() = ToConstEigenTp(IC_[5]);

  for (int j = 6; j < consts::model::kDimConfig; j++) {
    // f = spatial force required for a unit qdd_j
    dynamics::SpatialVec f = ToConstEigenTp(IC_[j]) * ToConstEigenTp(S_[j]);

    M(j, j) = ToConstEigenTp(S_[j]).dot(f);

    // Propagate down the tree
    f = ToConstEigenTp(Xup_[j]).transpose() * f;
    int i = parents_[j];
    while (i > 5) {
      // in here f is expressed in frame {i}
      M(i, j) = ToConstEigenTp(S_[i]).dot(f);
      M(j, i) = M(i, j);

      // Propagate down the tree
      f = ToConstEigenTp(Xup_[i]).transpose() * f;
      i = parents_[i];
    }

    // Force on floating base
    M.block<6, 1>(0, j) = f;
    M.block<1, 6>(j, 0) = f.adjoint();
  }
  return true;
}

bool FBModel::ComputeGeneralizedGravityForce(DynamicsData &ret) {
  CompositeInertias();

  dynamics::SpatialVec aGravity;

  aGravity << 0, 0, 0, gravity_[0], gravity_[1], gravity_[2];
  ToEigenTp(ag_[5]) = ToConstEigenTp(Xup_[5]) * aGravity;

  // Gravity comp force is the same as force required to accelerate
  // oppostite gravity
  Eigen::Map<genf_t> G(ret.Cg.data());
  G.topRows<6>() = -ToConstEigenTp(IC_[5]) * ToConstEigenTp(ag_[5]);
  for (int i = 6; i < consts::model::kDimConfig; i++) {
    ToEigenTp(ag_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(ag_[parents_[i]]);

    // body and rotor
    G[i] = -ToConstEigenTp(S_[i]).dot(ToConstEigenTp(IC_[i]) * ToConstEigenTp(ag_[i]));
  }
  return true;
}

bool FBModel::ComputeGeneralizedCoriolisForce(DynamicsData &ret) {
  BiasAccelerations();

  dynamics::SpatialVec tmpsv;

  // Floating base force
  Matrix6 Ifb = ToConstEigenTp(Ibody_[5]);
  dynamics::ForceCrossProduct(tmpsv, ToConstEigenTp(v_[5]), Ifb * ToConstEigenTp(v_[5]));
  ToEigenTp(fvp_[5]) = Ifb * ToConstEigenTp(avp_[5]) + tmpsv;

  for (int i = 6; i < consts::model::kDimConfig; i++) {
    // Force on body i
    auto Ii = ToConstEigenTp(Ibody_[i]);
    auto vi = ToConstEigenTp(v_[i]);
    dynamics::ForceCrossProduct(tmpsv, vi, Ii * vi);
    ToEigenTp(fvp_[i]) = Ii * ToConstEigenTp(avp_[i]) + tmpsv;
  }

  for (int i = consts::model::kDimConfig - 1; i > 5; i--) {
    // Extract force along the joints
    ret.Cc[i] = ToConstEigenTp(S_[i]).dot(ToConstEigenTp(fvp_[i]));

    // Propage force down the tree
    ToEigenTp(fvp_[parents_[i]]) += ToConstEigenTp(Xup_[i]).transpose() * ToConstEigenTp(fvp_[i]);
  }

  // Force on floating base
  Eigen::Map<genf_t> Cqd(ret.Cc.data());
  Cqd.topRows<6>() = ToConstEigenTp(fvp_[5]);
  return true;
}

bool FBModel::ComputeContactJacobians(DynamicsData &ret) {
  ForwardKinematics();
  BiasAccelerations();

  for (int k = 0; k < foot_count_; k++) {
    Eigen::Map<jc_t> Jc_k(ret.Jc[k].data());
    auto Jcdqd_k = ToEigenTp(ret.Jcdqd[k]);
    Jc_k.setZero();
    Jcdqd_k.setZero();

    int i = gc_parent_[k];

    // Rotation to absolute coords
    Matrix3 Rai = ToConstEigenTp(Xa_[i]).block<3, 3>(0, 0).transpose();
    Matrix6 Xc;
    dynamics::BuildSpatialXform(Xc, Rai, ToConstEigenTp(gc_location_[k]));

    // Bias acceleration
    dynamics::SpatialVec ac = Xc * ToConstEigenTp(avp_[i]);
    dynamics::SpatialVec vc = Xc * ToConstEigenTp(v_[i]);

    // Correct to classical
    dynamics::SpatialToLinearAcceleration(Jcdqd_k, ac, vc);

    // rows for linear velcoity in the world
    Eigen::Matrix3Xd Xout = Xc.bottomRows<3>();

    // from tips to base
    while (i > 5) {
      Jc_k.col(i) = Xout * ToConstEigenTp(S_[i]);
      Xout = Xout * ToConstEigenTp(Xup_[i]);
      i = parents_[i];
    }
    Jc_k.leftCols<6>() = Xout;
  }
  return true;
}

bool FBModel::AddBase(Eigen::Ref<dynamics::SpatialInertia const> const &inertia) {
  if (curr_n_dof_) {
    throw std::runtime_error("Cannot add base multiple times!\n");
  }

  SdMatrix6f eye6;
  ToEigenTp(eye6) = Matrix6::Identity();

  SdMatrix6f zero6 = {};
  // the floating base has 6 DOFs

  curr_n_dof_ = 6;
  for (int i = 0; i < curr_n_dof_; i++) {
    parents_.push_back(0);
    joint_types_.push_back(dynamics::JointType::Nothing);  // doesn't actually matter
    joint_axes_.push_back(dynamics::CoordinateAxis::X);    // doesn't actually matter
    Xtree_.push_back(eye6);
    Ibody_.push_back(zero6);
    body_names_.push_back("N/A");
  }

  joint_types_[5] = dynamics::JointType::FloatingBase;
  ToEigenTp(Ibody_[5]) = inertia;
  body_names_[5] = "Floating Base";

  AddDynamicsVars(6);
  return true;
}

bool FBModel::AddBase(fpt_t const mass, Eigen::Ref<Vector3 const> const &com, dynamics::InertiaTensor const &I) {
  Matrix6 IS;
  dynamics::BuildSpatialInertia(IS, mass, com, I);
  return AddBase(IS);
}

int FBModel::AddFoot(int const body_id, Eigen::Ref<Vector3 const> const &location) {
  if (body_id >= curr_n_dof_) {
    throw std::runtime_error("AddFoot got invalid body_id: " + std::to_string(body_id) +
                             " current nDofs: " + std::to_string(curr_n_dof_) + "\n");
  }

  // std::cout << "pt-add: " << location.transpose() << "\n";
  gc_parent_.push_back(body_id);
  gc_location_.push_back(SdVector3f({location[0], location[1], location[2]}));

  return foot_count_++;
}

int FBModel::AddBody(Eigen::Ref<dynamics::SpatialInertia const> const &inertia, int const parent,
                     dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                     Eigen::Ref<Matrix6 const> const &Xtree) {
  if (parent >= curr_n_dof_) {
    throw std::runtime_error("AddBody got invalid parent: " + std::to_string(parent) +
                             " nDofs: " + std::to_string(curr_n_dof_) + "\n");
  }

  parents_.push_back(parent);
  joint_types_.push_back(joint_type);
  joint_axes_.push_back(joint_axis);
  SdMatrix6f xt, inr;
  ToEigenTp(xt) = Xtree;
  ToEigenTp(inr) = inertia;
  Xtree_.push_back(xt);
  Ibody_.push_back(inr);
  curr_n_dof_++;

  AddDynamicsVars(1);

  return curr_n_dof_;
}

bool FBModel::AddDynamicsVars(int count) {
  if (count != 1 && count != 6) {
    throw std::runtime_error(
        "AddDynamicsVars must be called with count=1 (joint) or count=6 "
        "(base).\n");
  }

  SdMatrix6f eye6;
  ToEigenTp(eye6).setIdentity();

  SdVector6f zero6 = {};
  SdMatrix6f zeroInertia = {};

  for (int i = 0; i < count; i++) {
    v_.push_back(zero6);
    // a_.push_back(zero6);
    avp_.push_back(zero6);
    c_.push_back(zero6);
    S_.push_back(zero6);
    // f_.push_back(zero6);
    fvp_.push_back(zero6);
    ag_.push_back(zero6);
    IC_.push_back(zeroInertia);
    Xup_.push_back(eye6);
    Xa_.push_back(eye6);
  }

  return true;
}

bool FBModel::ResetCalculationFlags() {
  bias_acc_uptodate_ = false;
  composite_inertias_uptodate_ = false;
  kinematics_uptodate_ = false;
  return true;
}

bool FBModel::CompositeInertias() {
  if (composite_inertias_uptodate_) return false;

  ForwardKinematics();
  // initialize
  for (int i = 5; i < consts::model::kDimConfig; i++) {
    IC_[i] = Ibody_[i];
  }

  // backward loop
  for (int i = consts::model::kDimConfig - 1; i > 5; i--) {
    // Propagate inertia down the tree
    ToEigenTp(IC_[parents_[i]]) +=
        ToConstEigenTp(Xup_[i]).transpose() * ToConstEigenTp(IC_[i]) * ToConstEigenTp(Xup_[i]);
  }
  composite_inertias_uptodate_ = true;
  return true;
}

bool FBModel::BiasAccelerations() {
  if (bias_acc_uptodate_) return false;
  ForwardKinematics();
  // velocity product acceelration of base
  ToEigenTp(avp_[5]) << 0, 0, 0, 0, 0, 0;

  // from base to tips
  for (int i = 6; i < consts::model::kDimConfig; i++) {
    // Outward kinamtic propagtion
    ToEigenTp(avp_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(avp_[parents_[i]]) + ToConstEigenTp(c_[i]);
  }
  bias_acc_uptodate_ = true;

  return true;
}

bool FBModel::ForwardKinematics() {
  if (kinematics_uptodate_) return false;

  // calculate joint transformations
  dynamics::BuildSpatialXform(ToEigenTp(Xup_[5]), ToConstEigenTp(state_.rot_mat), ToConstEigenTp(state_.pos));
  v_[5] = state_.gvel_robot;

  for (int i = 6; i < consts::model::kDimConfig; i++) {
    // joint xform
    Matrix6 XJ;
    dynamics::BuildJointXform(XJ, joint_types_[i], joint_axes_[i], state_.q[i - 6]);
    ToEigenTp(Xup_[i]) = XJ * ToConstEigenTp(Xtree_[i]);
    dynamics::BuildJointMotionSubspace(ToEigenTp(S_[i]), joint_types_[i], joint_axes_[i]);
    dynamics::SpatialVec vJ = ToConstEigenTp(S_[i]) * state_.qd[i - 6];
    // total velocity of body i
    ToEigenTp(v_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(v_[parents_[i]]) + vJ;

    // Coriolis accelerations
    dynamics::MotionCrossProduct(ToEigenTp(c_[i]), ToConstEigenTp(v_[i]), vJ);
  }

  // calculate from absolute transformations
  for (int i = 5; i < consts::model::kDimConfig; i++) {
    if (parents_[i] == 0) {
      Xa_[i] = Xup_[i];  // float base
    } else {
      ToEigenTp(Xa_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(Xa_[parents_[i]]);
    }
  }
  kinematics_uptodate_ = true;
  return true;
}

}  // namespace sdquadx::model
