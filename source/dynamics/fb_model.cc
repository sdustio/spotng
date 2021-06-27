/*! @file fb_model.cc
 *  @brief Implementation of Rigid Body Floating Base model data structure
 *
 * This class stores the kinematic tree described in "Rigid Body Dynamics
 * Algorithms" by Featherstone (download from
 * https://www.springer.com/us/book/9780387743141 on MIT internet)
 *
 * The tree includes an additional "rotor" body for each body.  This rotor is
 * fixed to the parent body and has a gearing constraint.  This is efficiently
 * included using a technique similar to what is described in Chapter 12 of
 * "Robot and Multibody Dynamics" by Jain.  Note that this implementation is
 * highly specific to the case of a single rotating rotor per rigid body. Rotors
 * have the same joint type as their body, but with an additional gear ratio
 * multiplier applied to the motion subspace. The rotors associated with the
 * floating base don't do anything.
 */

#include <string>

#include "sd/dynamics/fb_model.h"
#include "sd/dynamics/rotation.h"
#include "sd/dynamics/spatial.h"
#include "sd/dynamics/inertia.h"

namespace sd::dynamics
{
  double FBModel::ApplyTestForce(const int gc_index,
                                 const Vector3d &force_ics_at_contact,
                                 VectorXd &dstate_out)
  {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();
    UdpateQddEffects();

    int i_opsp = gc_parent_.at(gc_index);
    int i = i_opsp;

    dstate_out = VectorXd::Zero(n_dof_);

    // Rotation to absolute coords
    Matrix3d Rai = Xa_[i].block<3, 3>(0, 0).transpose();
    Matrix6d Xc = CreateSpatialXform(Rai, gc_location_.at(gc_index));

    // D is one column of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SpatialVec F = Xc.transpose().rightCols<3>() * force_ics_at_contact;

    double LambdaInv = 0;
    double tmp = 0;

    // from tips to base
    while (i > 5)
    {
      tmp = F.dot(S_[i]);
      LambdaInv += tmp * tmp / d_[i];
      dstate_out.tail(n_dof_ - 6) += qdd_from_subqdd_.col(i - 6) * tmp / d_[i];

      // Apply force propagator (see Pat's ICRA 2012 paper)
      // essentially, since the joint is articulated, only a portion of the force
      // is felt on the predecessor. So, while Xup^T sends a force backwards as if
      // the joint was locked, ChiUp^T sends the force backward as if the joint
      // were free
      F = ChiUp_[i].transpose() * F;
      i = parents_[i];
    }

    dstate_out.head(6) = invIA5_.solve(F);
    LambdaInv += F.dot(dstate_out.head(6));
    dstate_out.tail(n_dof_ - 6) += qdd_from_base_acc_ * dstate_out.head(6);

    return LambdaInv;
  }

  double FBModel::ApplyTestForce(const int gc_index,
                                 const Vector3d &force_ics_at_contact,
                                 FBModelStateDerivative &dstate_out)
  {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();
    UdpateQddEffects();

    int i_opsp = gc_parent_.at(gc_index);
    int i = i_opsp;

    dstate_out.qdd.setZero();

    // Rotation to absolute coords
    Matrix3d Rai = Xa_[i].block<3, 3>(0, 0).transpose();
    Matrix6d Xc = CreateSpatialXform(Rai, gc_location_.at(gc_index));

    // D is one column of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SpatialVec F = Xc.transpose().rightCols<3>() * force_ics_at_contact;

    double LambdaInv = 0;
    double tmp = 0;

    // from tips to base
    while (i > 5)
    {
      tmp = F.dot(S_[i]);
      LambdaInv += tmp * tmp / d_[i];
      dstate_out.qdd += qdd_from_subqdd_.col(i - 6) * tmp / d_[i];

      // Apply force propagator (see Pat's ICRA 2012 paper)
      // essentially, since the joint is articulated, only a portion of the force
      // is felt on the predecessor. So, while Xup^T sends a force backwards as if
      // the joint was locked, ChiUp^T sends the force backward as if the joint
      // were free
      F = ChiUp_[i].transpose() * F;
      i = parents_[i];
    }

    // TODO: Only carry out the QR once within update Aritculated Bodies
    dstate_out.body_velocity_d = invIA5_.solve(F);
    LambdaInv += F.dot(dstate_out.body_velocity_d);
    dstate_out.qdd += qdd_from_base_acc_ * dstate_out.body_velocity_d;

    return LambdaInv;
  }

  void FBModel::UdpateQddEffects()
  {
    if (qdd_effects_uptodate_)
      return;
    UpdateForcePropagators();
    qdd_from_base_acc_.setZero();
    qdd_from_subqdd_.setZero();

    // Pass for force props
    // This loop is semi-equivalent to a cholesky factorization on H
    // akin to Featherstone's sparse operational space algo
    // These computations are for treating the joint rates like a task space
    // To do so, F computes the dynamic effect of torues onto bodies down the tree
    //
    for (int i = 6; i < n_dof_; i++)
    {
      qdd_from_subqdd_(i - 6, i - 6) = 1;
      SpatialVec F = (ChiUp_[i].transpose() - Xup_[i].transpose()) * S_[i];
      int j = parents_[i];
      while (j > 5)
      {
        qdd_from_subqdd_(i - 6, j - 6) = S_[j].dot(F);
        F = ChiUp_[j].transpose() * F;
        j = parents_[j];
      }
      qdd_from_base_acc_.row(i - 6) = F.transpose();
    }
    qdd_effects_uptodate_ = true;
  }

  void FBModel::UpdateForcePropagators()
  {
    if (force_propagators_uptodate_)
      return;
    UpdateArticulatedBodies();
    for (int i = 6; i < n_dof_; i++)
    {
      ChiUp_[i] = Xup_[i] - S_[i] * Utot_[i].transpose() / d_[i];
    }
    force_propagators_uptodate_ = true;
  }

  void FBModel::UpdateArticulatedBodies()
  {
    if (articulated_bodies_uptodate_)
      return;

    ForwardKinematics();

    IA_[5] = Ibody_[5];

    // loop 1, down the tree
    for (int i = 6; i < n_dof_; i++)
    {
      IA_[i] = Ibody_[i]; // initialize
      Matrix6d XJrot = JointXform(joint_types_[i], joint_axes_[i],
                                  state_.q[i - 6] * gear_ratios_[i]);
      Xuprot_[i] = XJrot * Xrot_[i];
      Srot_[i] = S_[i] * gear_ratios_[i];
    }

    // Pat's magic principle of least constraint (Guass too!)
    for (int i = n_dof_ - 1; i >= 6; i--)
    {
      U_[i] = IA_[i] * S_[i];
      Urot_[i] = Irot_[i] * Srot_[i];
      Utot_[i] = Xup_[i].transpose() * U_[i] + Xuprot_[i].transpose() * Urot_[i];

      d_[i] = Srot_[i].transpose() * Urot_[i];
      d_[i] += S_[i].transpose() * U_[i];

      // articulated inertia recursion
      Matrix6d Ia = Xup_[i].transpose() * IA_[i] * Xup_[i] +
                    Xuprot_[i].transpose() * Irot_[i] * Xuprot_[i] -
                    Utot_[i] * Utot_[i].transpose() / d_[i];
      IA_[parents_[i]] += Ia;
    }

    invIA5_.compute(IA_[5]);
    articulated_bodies_uptodate_ = true;
  }

  // parents, gr, jtype, Xtree, I, Xrot, Irot,

  void FBModel::AddDynamicsVars(int count)
  {
    if (count != 1 && count != 6)
    {
      throw std::runtime_error(
          "AddDynamicsVars must be called with count=1 (joint) or count=6 "
          "(base).\n");
    }

    Matrix6d eye6 = Matrix6d::Identity();
    SpatialVec zero6 = SpatialVec::Zero();

    SpatialInertia zeroInertia(Matrix6d::Zero());
    for (int i = 0; i < count; i++)
    {
      v_.push_back(zero6);
      vrot_.push_back(zero6);
      a_.push_back(zero6);
      arot_.push_back(zero6);
      avp_.push_back(zero6);
      avprot_.push_back(zero6);
      c_.push_back(zero6);
      crot_.push_back(zero6);
      S_.push_back(zero6);
      Srot_.push_back(zero6);
      f_.push_back(zero6);
      frot_.push_back(zero6);
      fvp_.push_back(zero6);
      fvprot_.push_back(zero6);
      ag_.push_back(zero6);
      agrot_.push_back(zero6);
      IC_.push_back(zeroInertia);
      Xup_.push_back(eye6);
      Xuprot_.push_back(eye6);
      Xa_.push_back(eye6);

      ChiUp_.push_back(eye6);
      d_.push_back(0.);
      u_.push_back(0.);
      IA_.push_back(eye6);

      U_.push_back(zero6);
      Urot_.push_back(zero6);
      Utot_.push_back(zero6);
      pA_.push_back(zero6);
      pArot_.push_back(zero6);
      external_forces_.push_back(zero6);
    }

    J_.push_back(SpatialVecXd::Zero(6, n_dof_));
    Jdqd_.push_back(SpatialVec::Zero());

    ResizeSystemMatricies();
  }

  void FBModel::ResizeSystemMatricies()
  {
    H_.setZero(n_dof_, n_dof_);
    C_.setZero(n_dof_, n_dof_);
    Cqd_.setZero(n_dof_);
    G_.setZero(n_dof_);
    for (unsigned i = 0; i < J_.size(); i++)
    {
      J_[i].setZero(6, n_dof_);
      Jdqd_[i].setZero();
    }

    for (unsigned i = 0; i < Jc_.size(); i++)
    {
      Jc_[i].setZero(3, n_dof_);
      Jcdqd_[i].setZero();
    }
    qdd_from_subqdd_.resize(n_dof_ - 6, n_dof_ - 6);
    qdd_from_base_acc_.resize(n_dof_ - 6, 6);
    state_.q = VectorXd::Zero(n_dof_ - 6);
    state_.qd = VectorXd::Zero(n_dof_ - 6);
  }

  void FBModel::AddBase(const SpatialInertia &inertia)
  {
    if (n_dof_)
    {
      throw std::runtime_error("Cannot add base multiple times!\n");
    }

    Matrix6d eye6 = Matrix6d::Identity();
    SpatialInertia zeroInertia(Matrix6d::Zero());
    // the floating base has 6 DOFs

    n_dof_ = 6;
    for (int i = 0; i < 6; i++)
    {
      parents_.push_back(0);
      gear_ratios_.push_back(0);
      joint_types_.push_back(JointType::Nothing); // doesn't actually matter
      joint_axes_.push_back(CoordinateAxis::X);   // doesn't actually matter
      Xtree_.push_back(eye6);
      Ibody_.push_back(zeroInertia);
      Xrot_.push_back(eye6);
      Irot_.push_back(zeroInertia);
      body_names_.push_back("N/A");
    }

    joint_types_[5] = JointType::FloatingBase;
    Ibody_[5] = inertia;
    gear_ratios_[5] = 1;
    body_names_[5] = "Floating Base";

    AddDynamicsVars(6);
  }

  void FBModel::AddBase(double mass, const Vector3d &com,
                        const Matrix3d &I)
  {
    SpatialInertia IS = BuildSpatialInertia(mass, com, I);
    AddBase(IS);
  }

  int FBModel::AddGroundContactPoint(int bodyID,
                                     const Vector3d &location,
                                     bool isFoot)
  {
    if (bodyID >= n_dof_)
    {
      throw std::runtime_error(
          "AddGroundContactPoint got invalid bodyID: " + std::to_string(bodyID) +
          " nDofs: " + std::to_string(n_dof_) + "\n");
    }

    // std::cout << "pt-add: " << location.transpose() << "\n";
    gc_parent_.push_back(bodyID);
    gc_location_.push_back(location);

    Vector3d zero3 = Vector3d::Zero();

    gc_p_.push_back(zero3);
    gc_v_.push_back(zero3);

    CartesianVecXd J(3, n_dof_);
    J.setZero();

    Jc_.push_back(J);
    Jcdqd_.push_back(zero3);
    //compute_contact_info_.push_back(false);
    compute_contact_info_.push_back(true);

    // add foot to foot list
    if (isFoot)
    {
      gc_foot_indices_.push_back(n_ground_contact_);
      compute_contact_info_[n_ground_contact_] = true;
    }

    ResizeSystemMatricies();
    return n_ground_contact_++;
  }

  void FBModel::AddGroundContactBoxPoints(int bodyId,
                                          const Vector3d &dims)
  {
    AddGroundContactPoint(bodyId, Vector3d(dims(0), dims(1), dims(2)) / 2);
    AddGroundContactPoint(bodyId, Vector3d(-dims(0), dims(1), dims(2)) / 2);
    AddGroundContactPoint(bodyId, Vector3d(dims(0), -dims(1), dims(2)) / 2);
    AddGroundContactPoint(bodyId, Vector3d(-dims(0), -dims(1), dims(2)) / 2);

    //AddGroundContactPoint(bodyId, Vector3d(dims(0), dims(1), 0.) / 2);
    //AddGroundContactPoint(bodyId, Vector3d(-dims(0), dims(1), 0.) / 2);
    //AddGroundContactPoint(bodyId, Vector3d(dims(0), -dims(1), 0.) / 2);
    //AddGroundContactPoint(bodyId, Vector3d(-dims(0), -dims(1), 0.) / 2);

    AddGroundContactPoint(bodyId, Vector3d(dims(0), dims(1), -dims(2)) / 2);
    AddGroundContactPoint(bodyId, Vector3d(-dims(0), dims(1), -dims(2)) / 2);
    AddGroundContactPoint(bodyId, Vector3d(dims(0), -dims(1), -dims(2)) / 2);
    AddGroundContactPoint(bodyId, Vector3d(-dims(0), -dims(1), -dims(2)) / 2);
  }

  int FBModel::AddBody(const SpatialInertia &inertia,
                       const SpatialInertia &rotor_inertia,
                       double gear_ratio, int parent, JointType joint_type,
                       CoordinateAxis joint_axis,
                       const Matrix6d &Xtree, const Matrix6d &Xrot)
  {
    if (parent >= n_dof_)
    {
      throw std::runtime_error(
          "AddBody got invalid parent: " + std::to_string(parent) +
          " nDofs: " + std::to_string(n_dof_) + "\n");
    }

    parents_.push_back(parent);
    gear_ratios_.push_back(gear_ratio);
    joint_types_.push_back(joint_type);
    joint_axes_.push_back(joint_axis);
    Xtree_.push_back(Xtree);
    Xrot_.push_back(Xrot);
    Ibody_.push_back(inertia);
    Irot_.push_back(rotor_inertia);
    n_dof_++;

    AddDynamicsVars(1);

    return n_dof_;
  }

  int FBModel::AddBody(const MassProperties &inertia,
                       const MassProperties &rotor_inertia,
                       double gear_ratio, int parent, JointType joint_type,
                       CoordinateAxis joint_axis,
                       const Matrix6d &Xtree, const Matrix6d &Xrot)
  {
    return AddBody(
        MassPropertiesToSpatialInertia(inertia),
        MassPropertiesToSpatialInertia(rotor_inertia),
        gear_ratio, parent, joint_type, joint_axis, Xtree, Xrot);
  }

  void FBModel::Check()
  {
    if (unsigned(n_dof_) != parents_.size())
      throw std::runtime_error("Invalid dof and parents length");
  }

  double FBModel::TotalNonRotorMass() const
  {
    double totalMass = 0;
    for (int i = 0; i < n_dof_; i++)
    {
      totalMass += MassFromSpatialInertia(Ibody_[i]);
    }
    return totalMass;
  }

  double FBModel::TotalRotorMass() const
  {
    double totalMass = 0;
    for (int i = 0; i < n_dof_; i++)
    {
      totalMass += MassFromSpatialInertia(Irot_[i]);
    }
    return totalMass;
  }

  void FBModel::ForwardKinematics()
  {
    if (kinematics_uptodate_)
      return;

    // calculate joint transformations
    Xup_[5] = CreateSpatialXform(QuatToRotMat(state_.body_orientation),
                                 state_.body_position);
    v_[5] = state_.body_velocity;
    for (int i = 6; i < n_dof_; i++)
    {
      // joint xform
      Matrix6d XJ = JointXform(joint_types_[i], joint_axes_[i], state_.q[i - 6]);
      Xup_[i] = XJ * Xtree_[i];
      S_[i] = JointMotionSubspace(joint_types_[i], joint_axes_[i]);
      SpatialVec vJ = S_[i] * state_.qd[i - 6];
      // total velocity of body i
      v_[i] = Xup_[i] * v_[parents_[i]] + vJ;

      // Same for rotors
      Matrix6d XJrot = JointXform(joint_types_[i], joint_axes_[i],
                                  state_.q[i - 6] * gear_ratios_[i]);
      Srot_[i] = S_[i] * gear_ratios_[i];
      SpatialVec vJrot = Srot_[i] * state_.qd[i - 6];
      Xuprot_[i] = XJrot * Xrot_[i];
      vrot_[i] = Xuprot_[i] * v_[parents_[i]] + vJrot;

      // Coriolis accelerations
      c_[i] = MotionCrossProduct(v_[i], vJ);
      crot_[i] = MotionCrossProduct(vrot_[i], vJrot);
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
        Xa_[i] = Xup_[i] * Xa_[parents_[i]];
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
      Matrix6d Xai = InvertSpatialXform(Xa_[i]); // from link to absolute
      SpatialVec vSpatial = Xai * v_[i];

      // foot position in world
      gc_p_[j] = SpatialXformPoint(Xai, gc_location_[j]);
      gc_v_[j] = SpatialToLinearVelocity(vSpatial, gc_p_[j]);
    }
    kinematics_uptodate_ = true;
  }

  void FBModel::ContactJacobians()
  {
    ForwardKinematics();
    BiasAccelerations();

    for (int k = 0; k < n_ground_contact_; k++)
    {
      Jc_[k].setZero();
      Jcdqd_[k].setZero();

      // Skip it if we don't care about it
      if (!compute_contact_info_[k])
        continue;

      int i = gc_parent_[k];

      // Rotation to absolute coords
      Matrix3d Rai = Xa_[i].block<3, 3>(0, 0).transpose();
      Matrix6d Xc = CreateSpatialXform(Rai, gc_location_[k]);

      // Bias acceleration
      SpatialVec ac = Xc * avp_[i];
      SpatialVec vc = Xc * v_[i];

      // Correct to classical
      Jcdqd_[k] = SpatialToLinearAcceleration(ac, vc);

      // rows for linear velcoity in the world
      CartesianVecXd Xout = Xc.bottomRows<3>();

      // from tips to base
      while (i > 5)
      {
        Jc_[k].col(i) = Xout * S_[i];
        Xout = Xout * Xup_[i];
        i = parents_[i];
      }
      Jc_[k].leftCols<6>() = Xout;
    }
  }

  void FBModel::BiasAccelerations()
  {
    if (bias_acc_uptodate_)
      return;
    ForwardKinematics();
    // velocity product acceelration of base
    avp_[5] << 0, 0, 0, 0, 0, 0;

    // from base to tips
    for (int i = 6; i < n_dof_; i++)
    {
      // Outward kinamtic propagtion
      avp_[i] = Xup_[i] * avp_[parents_[i]] + c_[i];
      avprot_[i] = Xuprot_[i] * avp_[parents_[i]] + crot_[i];
    }
    bias_acc_uptodate_ = true;
  }

  VectorXd FBModel::GeneralizedGravityForce()
  {
    CompositeInertias();

    SpatialVec aGravity;
    aGravity << 0, 0, 0, gravity_[0], gravity_[1], gravity_[2];
    ag_[5] = Xup_[5] * aGravity;

    // Gravity comp force is the same as force required to accelerate
    // oppostite gravity
    G_.topRows<6>() = -IC_[5] * ag_[5];
    for (int i = 6; i < n_dof_; i++)
    {
      ag_[i] = Xup_[i] * ag_[parents_[i]];
      agrot_[i] = Xuprot_[i] * ag_[parents_[i]];

      // body and rotor
      G_[i] = -S_[i].dot(IC_[i] * ag_[i]) -
              Srot_[i].dot(Irot_[i] * agrot_[i]);
    }
    return G_;
  }

  VectorXd FBModel::GeneralizedCoriolisForce()
  {
    BiasAccelerations();

    // Floating base force
    Matrix6d Ifb = Ibody_[5];
    SpatialVec hfb = Ifb * v_[5];
    fvp_[5] = Ifb * avp_[5] + ForceCrossProduct(v_[5], hfb);

    for (int i = 6; i < n_dof_; i++)
    {
      // Force on body i
      Matrix6d Ii = Ibody_[i];
      SpatialVec hi = Ii * v_[i];
      fvp_[i] = Ii * avp_[i] + ForceCrossProduct(v_[i], hi);

      // Force on rotor i
      Matrix6d Ir = Irot_[i];
      SpatialVec hr = Ir * vrot_[i];
      fvprot_[i] = Ir * avprot_[i] + ForceCrossProduct(vrot_[i], hr);
    }

    for (int i = n_dof_ - 1; i > 5; i--)
    {
      // Extract force along the joints
      Cqd_[i] = S_[i].dot(fvp_[i]) + Srot_[i].dot(fvprot_[i]);

      // Propage force down the tree
      fvp_[parents_[i]] += Xup_[i].transpose() * fvp_[i];
      fvp_[parents_[i]] += Xuprot_[i].transpose() * fvprot_[i];
    }

    // Force on floating base
    Cqd_.topRows<6>() = fvp_[5];
    return Cqd_;
  }

  Matrix3d FBModel::GetOrientation(int link_idx)
  {
    ForwardKinematics();
    Matrix3d Rai = Xa_[link_idx].block<3, 3>(0, 0);
    Rai.transposeInPlace();
    return Rai;
  }

  Vector3d FBModel::GetPosition(const int link_idx)
  {
    ForwardKinematics();
    Matrix6d Xai = InvertSpatialXform(Xa_[link_idx]); // from link to absolute
    Vector3d link_pos = SpatialXformPoint(Xai, Vector3d::Zero());
    return link_pos;
  }

  Vector3d FBModel::GetPosition(const int link_idx, const Vector3d &local_pos)
  {
    ForwardKinematics();
    Matrix6d Xai = InvertSpatialXform(Xa_[link_idx]); // from link to absolute
    Vector3d link_pos = SpatialXformPoint(Xai, local_pos);
    return link_pos;
  }

  Vector3d FBModel::GetLinearAcceleration(const int link_idx,
                                          const Vector3d &point)
  {
    ForwardAccelerationKinematics();
    Matrix3d R = GetOrientation(link_idx);
    return R * SpatialToLinearAcceleration(a_[link_idx], v_[link_idx], point);
  }

  Vector3d FBModel::GetLinearAcceleration(const int link_idx)
  {
    ForwardAccelerationKinematics();
    Matrix3d R = GetOrientation(link_idx);
    return R * SpatialToLinearAcceleration(a_[link_idx], v_[link_idx], Vector3d::Zero());
  }

  Vector3d FBModel::GetLinearVelocity(const int link_idx, const Vector3d &point)
  {
    ForwardKinematics();
    Matrix3d Rai = GetOrientation(link_idx);
    return Rai * SpatialToLinearVelocity(v_[link_idx], point);
  }

  Vector3d FBModel::GetLinearVelocity(const int link_idx)
  {
    ForwardKinematics();
    Matrix3d Rai = GetOrientation(link_idx);
    return Rai * SpatialToLinearVelocity(v_[link_idx], Vector3d::Zero());
  }

  Vector3d FBModel::GetAngularVelocity(const int link_idx)
  {
    ForwardKinematics();
    Matrix3d Rai = GetOrientation(link_idx);
    // Vector3d v3 =
    return Rai * v_[link_idx].head<3>();
    ;
  }

  Vector3d FBModel::GetAngularAcceleration(const int link_idx)
  {
    ForwardAccelerationKinematics();
    Matrix3d Rai = GetOrientation(link_idx);
    return Rai * a_[link_idx].head<3>();
  }

  void FBModel::CompositeInertias()
  {
    if (composite_inertias_uptodate_)
      return;

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
      IC_[parents_[i]] += Xup_[i].transpose() * IC_[i] * Xup_[i];
      IC_[parents_[i]] += Xuprot_[i].transpose() * Irot_[i] * Xuprot_[i];
    }
    composite_inertias_uptodate_ = true;
  }

  MatrixXd FBModel::GeneralizedMassMatrix()
  {
    CompositeInertias();
    H_.setZero();

    // Top left corner is the locked inertia of the whole system
    H_.topLeftCorner<6, 6>() = IC_[5];

    for (int j = 6; j < n_dof_; j++)
    {
      // f = spatial force required for a unit qdd_j
      SpatialVec f = IC_[j] * S_[j];
      SpatialVec frot = Irot_[j] * Srot_[j];

      H_(j, j) = S_[j].dot(f) + Srot_[j].dot(frot);

      // Propagate down the tree
      f = Xup_[j].transpose() * f + Xuprot_[j].transpose() * frot;
      int i = parents_[j];
      while (i > 5)
      {
        // in here f is expressed in frame {i}
        H_(i, j) = S_[i].dot(f);
        H_(j, i) = H_(i, j);

        // Propagate down the tree
        f = Xup_[i].transpose() * f;
        i = parents_[i];
      }

      // Force on floating base
      H_.block<6, 1>(0, j) = f;
      H_.block<1, 6>(j, 0) = f.adjoint();
    }
    return H_;
  }

  void FBModel::ForwardAccelerationKinematics()
  {
    if (acc_uptodate_)
    {
      return;
    }

    ForwardKinematics();
    BiasAccelerations();

    // Initialize gravity with model info
    SpatialVec aGravity = SpatialVec::Zero();
    aGravity.tail<3>() = gravity_;

    // Spatial force for floating base
    a_[5] = -Xup_[5] * aGravity + dstate_.body_velocity_d;

    // loop through joints
    for (int i = 6; i < n_dof_; i++)
    {
      // spatial acceleration
      a_[i] = Xup_[i] * a_[parents_[i]] + S_[i] * dstate_.qdd[i - 6] + c_[i];
      arot_[i] =
          Xuprot_[i] * a_[parents_[i]] + Srot_[i] * dstate_.qdd[i - 6] + crot_[i];
    }
    acc_uptodate_ = true;
  }

  VectorXd FBModel::InverseDynamics(const FBModelStateDerivative &dstate)
  {
    SetDState(dstate);
    ForwardAccelerationKinematics();

    // Spatial force for floating base
    SpatialVec hb = Ibody_[5] * v_[5];
    f_[5] = Ibody_[5] * a_[5] + ForceCrossProduct(v_[5], hb);

    // loop through joints
    for (int i = 6; i < n_dof_; i++)
    {
      // spatial momentum
      SpatialVec hi = Ibody_[i] * v_[i];
      SpatialVec hr = Irot_[i] * vrot_[i];

      // spatial force
      f_[i] = Ibody_[i] * a_[i] + ForceCrossProduct(v_[i], hi);
      frot_[i] =
          Irot_[i] * arot_[i] + ForceCrossProduct(vrot_[i], hr);
    }

    VectorXd genForce(n_dof_);
    for (int i = n_dof_ - 1; i > 5; i--)
    {
      // Pull off compoents of force along the joint
      genForce[i] = S_[i].dot(f_[i]) + Srot_[i].dot(frot_[i]);

      // Propagate down the tree
      f_[parents_[i]] += Xup_[i].transpose() * f_[i];
      f_[parents_[i]] += Xuprot_[i].transpose() * frot_[i];
    }
    genForce.head<6>() = f_[5];
    return genForce;
  }

  void FBModel::RunABA(const VectorXd &tau, FBModelStateDerivative &dstate)
  {
    (void)tau;
    ForwardKinematics();
    UpdateArticulatedBodies();

    // create spatial vector for gravity
    SpatialVec aGravity;
    aGravity << 0, 0, 0, gravity_[0], gravity_[1], gravity_[2];

    // float-base articulated inertia
    SpatialVec ivProduct = Ibody_[5] * v_[5];
    pA_[5] = ForceCrossProduct(v_[5], ivProduct);

    // loop 1, down the tree
    for (int i = 6; i < n_dof_; i++)
    {
      ivProduct = Ibody_[i] * v_[i];
      pA_[i] = ForceCrossProduct(v_[i], ivProduct);

      // same for rotors
      SpatialVec vJrot = Srot_[i] * state_.qd[i - 6];
      vrot_[i] = Xuprot_[i] * v_[parents_[i]] + vJrot;
      crot_[i] = MotionCrossProduct(vrot_[i], vJrot);
      ivProduct = Irot_[i] * vrot_[i];
      pArot_[i] = ForceCrossProduct(vrot_[i], ivProduct);
    }

    // adjust pA for external forces
    for (int i = 5; i < n_dof_; i++)
    {
      // TODO add if statement (avoid these calculations if the force is zero)
      Matrix3d R = RotationFromSpatialXform(Xa_[i]);
      Vector3d p = TranslationFromSpatialXform(Xa_[i]);
      Matrix6d iX = CreateSpatialXform(R.transpose(), -R * p);
      pA_[i] = pA_[i] - iX.transpose() * external_forces_[i];
    }

    // Pat's magic principle of least constraint
    for (int i = n_dof_ - 1; i >= 6; i--)
    {
      u_[i] = tau[i - 6] - S_[i].transpose() * pA_[i] -
              Srot_[i].transpose() * pArot_[i] - U_[i].transpose() * c_[i] -
              Urot_[i].transpose() * crot_[i];

      // articulated inertia recursion
      SpatialVec pa =
          Xup_[i].transpose() * (pA_[i] + IA_[i] * c_[i]) +
          Xuprot_[i].transpose() * (pArot_[i] + Irot_[i] * crot_[i]) +
          Utot_[i] * u_[i] / d_[i];
      pA_[parents_[i]] += pa;
    }

    // include gravity and compute acceleration of floating base
    SpatialVec a0 = -aGravity;
    SpatialVec ub = -pA_[5];
    a_[5] = Xup_[5] * a0;
    SpatialVec afb = invIA5_.solve(ub - IA_[5].transpose() * a_[5]);
    a_[5] += afb;

    // joint accelerations
    dstate.qdd = VectorXd(n_dof_ - 6);
    for (int i = 6; i < n_dof_; i++)
    {
      dstate.qdd[i - 6] =
          (u_[i] - Utot_[i].transpose() * a_[parents_[i]]) / d_[i];
      a_[i] = Xup_[i] * a_[parents_[i]] + S_[i] * dstate.qdd[i - 6] + c_[i];
    }

    // output
    RotMat Rup = RotationFromSpatialXform(Xup_[5]);
    dstate.body_position_d =
        Rup.transpose() * state_.body_velocity.block<3, 1>(3, 0);
    dstate.body_velocity_d = afb;
    // qdd is set in the for loop above
  }

  double FBModel::InvContactInertia(const int gc_index, const Vector3d &force_ics_at_contact)
  {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();

    int i_opsp = gc_parent_.at(gc_index);
    int i = i_opsp;

    // Rotation to absolute coords
    Matrix3d Rai = Xa_[i].block<3, 3>(0, 0).transpose();
    Matrix6d Xc = CreateSpatialXform(Rai, gc_location_.at(gc_index));

    // D is one column of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SpatialVec F = Xc.transpose().rightCols<3>() * force_ics_at_contact;

    double LambdaInv = 0;
    double tmp = 0;

    // from tips to base
    while (i > 5)
    {
      tmp = F.dot(S_[i]);
      LambdaInv += tmp * tmp / d_[i];

      // Apply force propagator (see Pat's ICRA 2012 paper)
      // essentially, since the joint is articulated, only a portion of the force
      // is felt on the predecessor. So, while Xup^T sends a force backwards as if
      // the joint was locked, ChiUp^T sends the force backward as if the joint
      // were free
      F = ChiUp_[i].transpose() * F;
      i = parents_[i];
    }
    LambdaInv += F.dot(invIA5_.solve(F));
    return LambdaInv;
  }

  MatrixXd FBModel::InvContactInertia(const int gc_index, const SpatialVecXd &force_directions)
  {
    ForwardKinematics();
    UpdateArticulatedBodies();
    UpdateForcePropagators();

    int i_opsp = gc_parent_.at(gc_index);
    int i = i_opsp;

    // Rotation to absolute coords
    Matrix3d Rai = Xa_[i].block<3, 3>(0, 0).transpose();
    Matrix6d Xc = CreateSpatialXform(Rai, gc_location_.at(gc_index));

    // D is a subslice of an extended force propagator matrix (See Wensing, 2012
    // ICRA)
    SpatialVecXd D = Xc.transpose() * force_directions;

    int m = force_directions.cols();

    MatrixXd LambdaInv = MatrixXd::Zero(m, m);
    VectorXd tmp = VectorXd::Zero(m);

    // from tips to base
    while (i > 5)
    {
      tmp = D.transpose() * S_[i];
      LambdaInv += tmp * tmp.transpose() / d_[i];

      // Apply force propagator (see Pat's ICRA 2012 paper)
      // essentially, since the joint is articulated, only a portion of the force
      // is felt on the predecessor. So, while Xup^T sends a force backwards as if
      // the joint was locked, ChiUp^T sends the force backward as if the joint
      // were free
      D = ChiUp_[i].transpose() * D;
      i = parents_[i];
    }

    // TODO: Only carry out the QR once within update Aritculated Bodies
    LambdaInv += D.transpose() * invIA5_.solve(D);

    return LambdaInv;
  }

  void FBModel::SetState(const FBModelState &state)
  {
    state_ = state;

    bias_acc_uptodate_ = false;
    composite_inertias_uptodate_ = false;

    ResetCalculationFlags();
  }

  void FBModel::ResetCalculationFlags()
  {
    articulated_bodies_uptodate_ = false;
    kinematics_uptodate_ = false;
    force_propagators_uptodate_ = false;
    qdd_effects_uptodate_ = false;
    acc_uptodate_ = false;
  }

  void FBModel::ResetExternalForces()
  {
    for (int i = 0; i < n_dof_; i++)
    {
      external_forces_[i] = SpatialVec::Zero();
    }
  }
}
