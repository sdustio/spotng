#include "model/float_base_impl.h"
#include "dynamics/inertia.h"
#include "dynamics/rotation.h"

namespace sdrobot::model
{
  bool FloatBaseModelImpl::UpdateState(FloatBaseModelState const &state)
  {
    state_ = state;
    ResetCalculationFlags();
  }

  bool FloatBaseModelImpl::UpdateGravity(SdVector3f const &g)
  {
    gravity_ = g;
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedGravityForce()
  {

    CompositeInertias();

    dynamics::SpatialVec aGravity;

    aGravity << 0, 0, 0, gravity_[0], gravity_[1], gravity_[2];
    ToEigenMatrix(ag_[5]) = ToConstEigenMatrix(Xup_[5]) * aGravity;

    // Gravity comp force is the same as force required to accelerate
    // oppostite gravity
    ToEigenMatrix(G_, n_dof_).topRows<6>() = -ToConstEigenMatrix(IC_[5]) * ToConstEigenMatrix(ag_[5]);
    for (int i = 6; i < n_dof_; i++)
    {
      ToEigenMatrix(ag_[i]) = ToConstEigenMatrix(Xup_[i]) * ToConstEigenMatrix(ag_[parents_[i]]);
      ToEigenMatrix(agrot_[i]) = ToConstEigenMatrix(Xuprot_[i]) * ToConstEigenMatrix(ag_[parents_[i]]);

      // body and rotor
      G_[i] = -ToConstEigenMatrix(S_[i]).dot(ToConstEigenMatrix(IC_[i]) * ToConstEigenMatrix(ag_[i])) -
              ToConstEigenMatrix(Srot_[i]).dot(ToConstEigenMatrix(Irot_[i]) * ToConstEigenMatrix(agrot_[i]));
    }
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedCoriolisForce()
  {
    BiasAccelerations();

    dynamics::SpatialVec tmpsv;

    // Floating base force
    Matrix6 Ifb = ToConstEigenMatrix(Ibody_[5]);
    dynamics::ForceCrossProduct(tmpsv, ToConstEigenMatrix(v_[5]), Ifb * ToConstEigenMatrix(v_[5]));
    ToEigenMatrix(fvp_[5]) = Ifb * ToConstEigenMatrix(avp_[5]) + tmpsv;

    for (int i = 6; i < n_dof_; i++)
    {
      // Force on body i
      auto Ii = ToConstEigenMatrix(Ibody_[i]);
      auto vi = ToConstEigenMatrix(v_[i]);
      dynamics::ForceCrossProduct(tmpsv, vi, Ii * vi);
      ToEigenMatrix(fvp_[i]) = Ii * ToConstEigenMatrix(avp_[i]) + tmpsv;

      // Force on rotor i
      auto Ir = ToConstEigenMatrix(Irot_[i]);
      auto vr = ToConstEigenMatrix(vrot_[i]);
      dynamics::ForceCrossProduct(tmpsv, vr, Ir * vr);
      ToEigenMatrix(fvprot_[i]) = Ir * ToConstEigenMatrix(avprot_[i]) + tmpsv;
    }

    for (int i = n_dof_ - 1; i > 5; i--)
    {
      // Extract force along the joints
      Cqd_[i] = ToConstEigenMatrix(S_[i]).dot(ToConstEigenMatrix(fvp_[i])) + ToConstEigenMatrix(Srot_[i]).dot(ToConstEigenMatrix(fvprot_[i]));

      // Propage force down the tree
      ToEigenMatrix(fvp_[parents_[i]]) += ToConstEigenMatrix(Xup_[i]).transpose() * ToConstEigenMatrix(fvp_[i]);
      ToEigenMatrix(fvp_[parents_[i]]) += ToConstEigenMatrix(Xuprot_[i]).transpose() * ToConstEigenMatrix(fvprot_[i]);
    }

    // Force on floating base
    ToEigenMatrix(Cqd_, n_dof_).topRows<6>() = ToConstEigenMatrix(fvp_[5]);
    return true;
  }

  bool FloatBaseModelImpl::ComputeContactJacobians()
  {
    ForwardKinematics();
    BiasAccelerations();

    for (int k = 0; k < n_ground_contact_; k++)
    {
      auto Jc_k = ToEigenMatrix(Jc_[k], 3, n_dof_);
      auto Jcdqd_k = ToEigenMatrix(Jcdqd_[k]);
      Jc_k.setZero();
      Jcdqd_k.setZero();

      // Skip it if we don't care about it
      if (!compute_contact_info_[k])
        continue;

      int i = gc_parent_[k];

      // Rotation to absolute coords
      Matrix3 Rai = ToConstEigenMatrix(Xa_[i]).block<3, 3>(0, 0).transpose();
      Matrix6 Xc;
      dynamics::BuildSpatialXform(Xc, Rai, ToConstEigenMatrix(gc_location_[k]));

      // Bias acceleration
      dynamics::SpatialVec ac = Xc * ToConstEigenMatrix(avp_[i]);
      dynamics::SpatialVec vc = Xc * ToConstEigenMatrix(v_[i]);

      // Correct to classical
      dynamics::SpatialToLinearAcceleration(Jcdqd_k, ac, vc);

      // rows for linear velcoity in the world
      Eigen::Matrix3Xd Xout = Xc.bottomRows<3>();

      // from tips to base
      while (i > 5)
      {
        Jc_k.col(i) = Xout * ToConstEigenMatrix(S_[i]);
        Xout = Xout * ToConstEigenMatrix(Xup_[i]);
        i = parents_[i];
      }
      Jc_k.leftCols<6>() = Xout;
    }
    return true;
  }

  bool FloatBaseModelImpl::AddBase(Eigen::Ref<dynamics::SpatialInertia const> const &inertia)
  {
  }

  bool FloatBaseModelImpl::AddBase(double const mass, Eigen::Ref<Vector3 const> const &com, dynamics::RotationalInertia const &I)
  {
  }
  int FloatBaseModelImpl::AddGroundContactPoint(int const body_id, Eigen::Ref<Vector3 const> const &location,
                                                bool const is_foot)
  {
  }

  void FloatBaseModelImpl::AddGroundContactBoxPoints(int const body_id, Eigen::Ref<Vector3 const> const &dims)
  {
  }
  int FloatBaseModelImpl::AddBody(dynamics::SpatialInertia const &inertia,
                                  dynamics::SpatialInertia const &rotor_inertia, double const gear_ratio, int const parent,
                                  dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                                  Matrix6 const &Xtree, Matrix6 const &Xrot)
  {
  }
  int FloatBaseModelImpl::AddBody(dynamics::MassProperties const &inertia,
                                  dynamics::MassProperties const &rotor_inertia, double const gear_ratio, int const parent,
                                  dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                                  Matrix6 const &Xtree, Matrix6 const &Xrot)
  {
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
    for (int i = 5; i < n_dof_; i++)
    {
      IC_[i] = Ibody_[i];
    }

    // backward loop
    for (int i = n_dof_ - 1; i > 5; i--)
    {
      // Propagate inertia down the tree
      ToEigenMatrix(IC_[parents_[i]]) += ToConstEigenMatrix(Xup_[i]).transpose() * ToConstEigenMatrix(IC_[i]) * ToConstEigenMatrix(Xup_[i]);
      ToEigenMatrix(IC_[parents_[i]]) += ToConstEigenMatrix(Xuprot_[i]).transpose() * ToConstEigenMatrix(Irot_[i]) * ToConstEigenMatrix(Xuprot_[i]);
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
    ToEigenMatrix(avp_[5]) << 0, 0, 0, 0, 0, 0;

    // from base to tips
    for (int i = 6; i < n_dof_; i++)
    {
      // Outward kinamtic propagtion
      ToEigenMatrix(avp_[i]) = ToConstEigenMatrix(Xup_[i]) * ToConstEigenMatrix(avp_[parents_[i]]) + ToConstEigenMatrix(c_[i]);
      ToEigenMatrix(avprot_[i]) = ToConstEigenMatrix(Xuprot_[i]) * ToConstEigenMatrix(avp_[parents_[i]]) + ToConstEigenMatrix(crot_[i]);
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
    dynamics::QuatToRotMat(rot, ToConstEigenMatrix(state_.ori));
    dynamics::BuildSpatialXform(ToEigenMatrix(Xup_[5]), rot, ToConstEigenMatrix(state_.pos));
    v_[5] = state_.vel_body;

    for (int i = 6; i < n_dof_; i++)
    {
      // joint xform
      Matrix6 XJ;
      dynamics::JointXform(XJ, joint_types_[i], joint_axes_[i], state_.q[i - 6]);
      ToEigenMatrix(Xup_[i]) = XJ * ToConstEigenMatrix(Xtree_[i]);
      dynamics::JointMotionSubspace(ToEigenMatrix(S_[i]), joint_types_[i], joint_axes_[i]);
      dynamics::SpatialVec vJ = ToConstEigenMatrix(S_[i]) * state_.qd[i - 6];
      // total velocity of body i
      ToEigenMatrix(v_[i]) = ToConstEigenMatrix(Xup_[i]) * ToConstEigenMatrix(v_[parents_[i]]) + vJ;

      // Same for rotors
      Matrix6 XJrot;
      JointXform(XJrot, joint_types_[i], joint_axes_[i], state_.q[i - 6] * gear_ratios_[i]);
      ToEigenMatrix(Srot_[i]) = ToConstEigenMatrix(S_[i]) * gear_ratios_[i];
      dynamics::SpatialVec vJrot = ToConstEigenMatrix(Srot_[i]) * state_.qd[i - 6];
      ToEigenMatrix(Xuprot_[i]) = XJrot * ToConstEigenMatrix(Xrot_[i]);
      ToEigenMatrix(vrot_[i]) = ToConstEigenMatrix(Xuprot_[i]) * ToConstEigenMatrix(v_[parents_[i]]) + vJrot;

      // Coriolis accelerations
      dynamics::MotionCrossProduct(ToEigenMatrix(c_[i]), ToConstEigenMatrix(v_[i]), vJ);
      dynamics::MotionCrossProduct(ToEigenMatrix(crot_[i]), ToConstEigenMatrix(vrot_[i]), vJrot);
    }

    // calculate from absolute transformations
    for (int i = 5; i < n_dof_; i++)
    {
      if (parents_[i] == 0)
      {
        Xa_[i] = Xup_[i]; // float base
      }
      else
      {
        ToEigenMatrix(Xa_[i]) = ToConstEigenMatrix(Xup_[i]) * ToConstEigenMatrix(Xa_[parents_[i]]);
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
      dynamics::InvertSpatialXform(Xai, ToConstEigenMatrix(Xa_[i])); // from link to absolute
      dynamics::SpatialVec vSpatial = Xai * ToConstEigenMatrix(v_[i]);

      // foot position in world
      dynamics::SpatialXformPoint(ToEigenMatrix(gc_p_[j]), Xai, ToConstEigenMatrix(gc_location_[j]));
      dynamics::SpatialToLinearVelocity(ToEigenMatrix(gc_v_[j]), vSpatial, ToConstEigenMatrix(gc_p_[j]));
    }
    kinematics_uptodate_ = true;
    return true;
  }

}
