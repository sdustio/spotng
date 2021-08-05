#include "model/float_base_impl.h"
#include "dynamics/inertia.h"
#include "dynamics/rotation.h"

namespace sdrobot::model
{
  using genf_t = Eigen::Matrix<fpt_t, params::model::dim_config, 1>;
  using mass_t = Eigen::Matrix<fpt_t, params::model::dim_config, params::model::dim_config>;
  using jc_t = Eigen::Matrix<fpt_t, 3, params::model::dim_config>;

  bool FloatBaseModelImpl::UpdateState(FloatBaseModelState const &state)
  {
    state_ = state;
    ResetCalculationFlags();
    return true;
  }

  bool FloatBaseModelImpl::UpdateGravity(SdVector3f const &g)
  {
    gravity_ = g;
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedMassMatrix()
  {
    CompositeInertias();
    H_.fill(0.);

    Eigen::Map<mass_t> H(H_.data());

    // Top left corner is the locked inertia of the whole system
    H.topLeftCorner<6, 6>() = ToConstEigenTp(IC_[5]);

    for (int j = 6; j < params::model::dim_config; j++)
    {
      // f = spatial force required for a unit qdd_j
      dynamics::SpatialVec f = ToConstEigenTp(IC_[j]) * ToConstEigenTp(S_[j]);
      dynamics::SpatialVec frot = ToConstEigenTp(Irot_[j]) * ToConstEigenTp(Srot_[j]);

      H(j, j) = ToConstEigenTp(S_[j]).dot(f) + ToConstEigenTp(Srot_[j]).dot(frot);

      // Propagate down the tree
      f = ToConstEigenTp(Xup_[j]).transpose() * f + ToConstEigenTp(Xuprot_[j]).transpose() * frot;
      int i = parents_[j];
      while (i > 5)
      {
        // in here f is expressed in frame {i}
        H(i, j) = ToConstEigenTp(S_[i]).dot(f);
        H(j, i) = H(i, j);

        // Propagate down the tree
        f = ToConstEigenTp(Xup_[i]).transpose() * f;
        i = parents_[i];
      }

      // Force on floating base
      H.block<6, 1>(0, j) = f;
      H.block<1, 6>(j, 0) = f.adjoint();
    }
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedGravityForce()
  {

    CompositeInertias();

    dynamics::SpatialVec aGravity;

    aGravity << 0, 0, 0, gravity_[0], gravity_[1], gravity_[2];
    ToEigenTp(ag_[5]) = ToConstEigenTp(Xup_[5]) * aGravity;

    // Gravity comp force is the same as force required to accelerate
    // oppostite gravity
    Eigen::Map<genf_t> G(G_.data());
    G.topRows<6>() = -ToConstEigenTp(IC_[5]) * ToConstEigenTp(ag_[5]);
    for (int i = 6; i < params::model::dim_config; i++)
    {
      ToEigenTp(ag_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(ag_[parents_[i]]);
      ToEigenTp(agrot_[i]) = ToConstEigenTp(Xuprot_[i]) * ToConstEigenTp(ag_[parents_[i]]);

      // body and rotor
      G[i] = -ToConstEigenTp(S_[i]).dot(ToConstEigenTp(IC_[i]) * ToConstEigenTp(ag_[i])) -
              ToConstEigenTp(Srot_[i]).dot(ToConstEigenTp(Irot_[i]) * ToConstEigenTp(agrot_[i]));
    }
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedCoriolisForce()
  {
    BiasAccelerations();

    dynamics::SpatialVec tmpsv;

    // Floating base force
    Matrix6 Ifb = ToConstEigenTp(Ibody_[5]);
    dynamics::ForceCrossProduct(tmpsv, ToConstEigenTp(v_[5]), Ifb * ToConstEigenTp(v_[5]));
    ToEigenTp(fvp_[5]) = Ifb * ToConstEigenTp(avp_[5]) + tmpsv;

    for (int i = 6; i < params::model::dim_config; i++)
    {
      // Force on body i
      auto Ii = ToConstEigenTp(Ibody_[i]);
      auto vi = ToConstEigenTp(v_[i]);
      dynamics::ForceCrossProduct(tmpsv, vi, Ii * vi);
      ToEigenTp(fvp_[i]) = Ii * ToConstEigenTp(avp_[i]) + tmpsv;

      // Force on rotor i
      auto Ir = ToConstEigenTp(Irot_[i]);
      auto vr = ToConstEigenTp(vrot_[i]);
      dynamics::ForceCrossProduct(tmpsv, vr, Ir * vr);
      ToEigenTp(fvprot_[i]) = Ir * ToConstEigenTp(avprot_[i]) + tmpsv;
    }

    for (int i = params::model::dim_config - 1; i > 5; i--)
    {
      // Extract force along the joints
      Cqd_[i] = ToConstEigenTp(S_[i]).dot(ToConstEigenTp(fvp_[i])) + ToConstEigenTp(Srot_[i]).dot(ToConstEigenTp(fvprot_[i]));

      // Propage force down the tree
      ToEigenTp(fvp_[parents_[i]]) += ToConstEigenTp(Xup_[i]).transpose() * ToConstEigenTp(fvp_[i]);
      ToEigenTp(fvp_[parents_[i]]) += ToConstEigenTp(Xuprot_[i]).transpose() * ToConstEigenTp(fvprot_[i]);
    }

    // Force on floating base
    Eigen::Map<genf_t> Cqd(Cqd_.data());
    Cqd.topRows<6>() = ToConstEigenTp(fvp_[5]);
    return true;
  }

  bool FloatBaseModelImpl::ComputeContactJacobians()
  {
    ForwardKinematics();
    BiasAccelerations();

    for (int k = 0; k < n_ground_contact_; k++)
    {
      Eigen::Map<jc_t> Jc_k(Jc_[k].data());
      auto Jcdqd_k = ToEigenTp(Jcdqd_[k]);
      Jc_k.setZero();
      Jcdqd_k.setZero();

      // Skip it if we don't care about it
      if (!compute_contact_info_[k])
        continue;

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
      while (i > 5)
      {
        Jc_k.col(i) = Xout * ToConstEigenTp(S_[i]);
        Xout = Xout * ToConstEigenTp(Xup_[i]);
        i = parents_[i];
      }
      Jc_k.leftCols<6>() = Xout;
    }
    return true;
  }

  bool FloatBaseModelImpl::AddBase(Eigen::Ref<dynamics::SpatialInertia const> const &inertia)
  {
    if (curr_n_dof_)
    {
      throw std::runtime_error("Cannot add base multiple times!\n");
    }

    SdMatrix6f eye6;
    ToEigenTp(eye6) = Matrix6::Identity();

    SdMatrix6f zero6 = {};
    // the floating base has 6 DOFs

    curr_n_dof_ = 6;
    for (int i = 0; i < curr_n_dof_; i++)
    {
      parents_.push_back(0);
      gear_ratios_.push_back(0);
      joint_types_.push_back(dynamics::JointType::Nothing); // doesn't actually matter
      joint_axes_.push_back(dynamics::CoordinateAxis::X);   // doesn't actually matter
      Xtree_.push_back(eye6);
      Ibody_.push_back(zero6);
      Xrot_.push_back(eye6);
      Irot_.push_back(zero6);
      body_names_.push_back("N/A");
    }

    joint_types_[5] = dynamics::JointType::FloatingBase;
    ToEigenTp(Ibody_[5]) = inertia;
    gear_ratios_[5] = 1;
    body_names_[5] = "Floating Base";

    AddDynamicsVars(6);
    return true;
  }

  bool FloatBaseModelImpl::AddBase(fpt_t const mass, Eigen::Ref<Vector3 const> const &com, dynamics::RotationalInertia const &I)
  {
    Matrix6 IS;
    dynamics::BuildSpatialInertia(IS, mass, com, I);
    return AddBase(IS);
  }

  int FloatBaseModelImpl::AddGroundContactPoint(int const body_id, Eigen::Ref<Vector3 const> const &location,
                                                bool const is_foot)
  {
    if (body_id >= curr_n_dof_)
    {
      throw std::runtime_error(
          "AddGroundContactPoint got invalid body_id: " + std::to_string(body_id) +
          " current nDofs: " + std::to_string(curr_n_dof_) + "\n");
    }

    // std::cout << "pt-add: " << location.transpose() << "\n";
    gc_parent_.push_back(body_id);
    gc_location_.push_back(SdVector3f({location[0], location[1], location[2]}));

    SdVector3f zero3 = {};

    gc_p_.push_back(zero3);
    gc_v_.push_back(zero3);

    ContactJacobTp J = {};

    Jc_.push_back(J);
    Jcdqd_.push_back(zero3);
    //compute_contact_info_.push_back(false);
    compute_contact_info_.push_back(true);

    // add foot to foot list
    if (is_foot)
    {
      gc_foot_indices_.push_back(n_ground_contact_);
      // compute_contact_info_[n_ground_contact_] = true;
    }

    return n_ground_contact_++;
  }

  void FloatBaseModelImpl::AddGroundContactBoxPoints(int const body_id, Eigen::Ref<Vector3 const> const &dims)
  {
    AddGroundContactPoint(body_id, Vector3(dims[0], dims[1], dims[2]) / 2);
    AddGroundContactPoint(body_id, Vector3(-dims[0], dims[1], dims[2]) / 2);
    AddGroundContactPoint(body_id, Vector3(dims[0], -dims[1], dims[2]) / 2);
    AddGroundContactPoint(body_id, Vector3(-dims[0], -dims[1], dims[2]) / 2);

    AddGroundContactPoint(body_id, Vector3(dims[0], dims[1], -dims[2]) / 2);
    AddGroundContactPoint(body_id, Vector3(-dims[0], dims[1], -dims[2]) / 2);
    AddGroundContactPoint(body_id, Vector3(dims[0], -dims[1], -dims[2]) / 2);
    AddGroundContactPoint(body_id, Vector3(-dims[0], -dims[1], -dims[2]) / 2);
  }

  int FloatBaseModelImpl::AddBody(Eigen::Ref<dynamics::SpatialInertia const> const &inertia,
                                  Eigen::Ref<dynamics::SpatialInertia const> const &rotor_inertia, fpt_t const gear_ratio, int const parent,
                                  dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                                  Eigen::Ref<Matrix6 const> const &Xtree, Eigen::Ref<Matrix6 const> const &Xrot)
  {
    if (parent >= curr_n_dof_)
    {
      throw std::runtime_error(
          "AddBody got invalid parent: " + std::to_string(parent) +
          " nDofs: " + std::to_string(curr_n_dof_) + "\n");
    }

    parents_.push_back(parent);
    gear_ratios_.push_back(gear_ratio);
    joint_types_.push_back(joint_type);
    joint_axes_.push_back(joint_axis);
    SdMatrix6f xt, xr, inr, rinr;
    ToEigenTp(xt) = Xtree;
    ToEigenTp(xr) = Xrot;
    ToEigenTp(inr) = inertia;
    ToEigenTp(rinr) = rotor_inertia;
    Xtree_.push_back(xt);
    Xrot_.push_back(xr);
    Ibody_.push_back(inr);
    Irot_.push_back(rinr);
    curr_n_dof_++;

    AddDynamicsVars(1);

    return curr_n_dof_;
  }

  void FloatBaseModelImpl::AddDynamicsVars(int count)
  {
    if (count != 1 && count != 6)
    {
      throw std::runtime_error(
          "AddDynamicsVars must be called with count=1 (joint) or count=6 "
          "(base).\n");
    }

    SdMatrix6f eye6;
    ToEigenTp(eye6).setIdentity();

    SdVector6f zero6 = {};
    SdMatrix6f zeroInertia = {};

    for (int i = 0; i < count; i++)
    {
      v_.push_back(zero6);
      vrot_.push_back(zero6);
      // a_.push_back(zero6);
      // arot_.push_back(zero6);
      avp_.push_back(zero6);
      avprot_.push_back(zero6);
      c_.push_back(zero6);
      crot_.push_back(zero6);
      S_.push_back(zero6);
      Srot_.push_back(zero6);
      // f_.push_back(zero6);
      // frot_.push_back(zero6);
      fvp_.push_back(zero6);
      fvprot_.push_back(zero6);
      ag_.push_back(zero6);
      agrot_.push_back(zero6);
      IC_.push_back(zeroInertia);
      Xup_.push_back(eye6);
      Xuprot_.push_back(eye6);
      Xa_.push_back(eye6);

      // ChiUp_.push_back(eye6);
      // d_.push_back(0.);
      // u_.push_back(0.);
      // IA_.push_back(eye6);

      // U_.push_back(zero6);
      // Urot_.push_back(zero6);
      // Utot_.push_back(zero6);
      // pA_.push_back(zero6);
      // pArot_.push_back(zero6);
      // external_forces_.push_back(zero6);
    }

    // J_.push_back(SpatialVecXd::Zero(6, curr_n_dof_));
    // Jdqd_.push_back(SpatialVec::Zero());
  }

  void FloatBaseModelImpl::ResetCalculationFlags()
  {
    bias_acc_uptodate_ = false;
    composite_inertias_uptodate_ = false;

    articulated_bodies_uptodate_ = false;
    kinematics_uptodate_ = false;
    force_propagators_uptodate_ = false;
    qdd_effects_uptodate_ = false;
    acc_uptodate_ = false;
  }

  bool FloatBaseModelImpl::CompositeInertias()
  {

    if (composite_inertias_uptodate_)
      return false;

    ForwardKinematics();
    // initialize
    for (int i = 5; i < params::model::dim_config; i++)
    {
      IC_[i] = Ibody_[i];
    }

    // backward loop
    for (int i = params::model::dim_config - 1; i > 5; i--)
    {
      // Propagate inertia down the tree
      ToEigenTp(IC_[parents_[i]]) += ToConstEigenTp(Xup_[i]).transpose() * ToConstEigenTp(IC_[i]) * ToConstEigenTp(Xup_[i]);
      ToEigenTp(IC_[parents_[i]]) += ToConstEigenTp(Xuprot_[i]).transpose() * ToConstEigenTp(Irot_[i]) * ToConstEigenTp(Xuprot_[i]);
    }
    composite_inertias_uptodate_ = true;
    return true;
  }

  bool FloatBaseModelImpl::BiasAccelerations()
  {
    if (bias_acc_uptodate_)
      return false;
    ForwardKinematics();
    // velocity product acceelration of base
    ToEigenTp(avp_[5]) << 0, 0, 0, 0, 0, 0;

    // from base to tips
    for (int i = 6; i < params::model::dim_config; i++)
    {
      // Outward kinamtic propagtion
      ToEigenTp(avp_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(avp_[parents_[i]]) + ToConstEigenTp(c_[i]);
      ToEigenTp(avprot_[i]) = ToConstEigenTp(Xuprot_[i]) * ToConstEigenTp(avp_[parents_[i]]) + ToConstEigenTp(crot_[i]);
    }
    bias_acc_uptodate_ = true;

    return true;
  }

  bool FloatBaseModelImpl::ForwardKinematics()
  {
    if (kinematics_uptodate_)
      return false;

    // calculate joint transformations
    dynamics::RotMat rot;
    dynamics::QuatToRotMat(rot, ToConstEigenTp(state_.ori));
    dynamics::BuildSpatialXform(ToEigenTp(Xup_[5]), rot, ToConstEigenTp(state_.pos));
    v_[5] = state_.vel;

    for (int i = 6; i < params::model::dim_config; i++)
    {
      // joint xform
      Matrix6 XJ;
      dynamics::BuildJointXform(XJ, joint_types_[i], joint_axes_[i], state_.q[i - 6]);
      ToEigenTp(Xup_[i]) = XJ * ToConstEigenTp(Xtree_[i]);
      dynamics::BuildJointMotionSubspace(ToEigenTp(S_[i]), joint_types_[i], joint_axes_[i]);
      dynamics::SpatialVec vJ = ToConstEigenTp(S_[i]) * state_.qd[i - 6];
      // total velocity of body i
      ToEigenTp(v_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(v_[parents_[i]]) + vJ;

      // Same for rotors
      Matrix6 XJrot;
      BuildJointXform(XJrot, joint_types_[i], joint_axes_[i], state_.q[i - 6] * gear_ratios_[i]);
      ToEigenTp(Srot_[i]) = ToConstEigenTp(S_[i]) * gear_ratios_[i];
      dynamics::SpatialVec vJrot = ToConstEigenTp(Srot_[i]) * state_.qd[i - 6];
      ToEigenTp(Xuprot_[i]) = XJrot * ToConstEigenTp(Xrot_[i]);
      ToEigenTp(vrot_[i]) = ToConstEigenTp(Xuprot_[i]) * ToConstEigenTp(v_[parents_[i]]) + vJrot;

      // Coriolis accelerations
      dynamics::MotionCrossProduct(ToEigenTp(c_[i]), ToConstEigenTp(v_[i]), vJ);
      dynamics::MotionCrossProduct(ToEigenTp(crot_[i]), ToConstEigenTp(vrot_[i]), vJrot);
    }

    // calculate from absolute transformations
    for (int i = 5; i < params::model::dim_config; i++)
    {
      if (parents_[i] == 0)
      {
        Xa_[i] = Xup_[i]; // float base
      }
      else
      {
        ToEigenTp(Xa_[i]) = ToConstEigenTp(Xup_[i]) * ToConstEigenTp(Xa_[parents_[i]]);
      }
    }

    // ground contact points
    //  // TODO : we end up inverting the same Xa a few times (like for the 8
    //  points on the body). this isn't super efficient.
    for (int j = 0; j < n_ground_contact_; j++)
    {
      if (!compute_contact_info_[j])
        continue;
      int i = gc_parent_[j];
      Matrix6 Xai;
      dynamics::InvertSpatialXform(Xai, ToConstEigenTp(Xa_[i])); // from link to absolute
      dynamics::SpatialVec vSpatial = Xai * ToConstEigenTp(v_[i]);

      // foot position in world
      dynamics::SpatialXformPoint(ToEigenTp(gc_p_[j]), Xai, ToConstEigenTp(gc_location_[j]));
      dynamics::SpatialToLinearVelocity(ToEigenTp(gc_v_[j]), vSpatial, ToConstEigenTp(gc_p_[j]));
    }
    kinematics_uptodate_ = true;
    return true;
  }

}
