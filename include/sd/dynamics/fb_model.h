#pragma once

#include <memory>
#include <vector>

#include "sd/dynamics/kinematics.h"

/*
 This class stores the kinematic tree described in "Rigid Body Dynamics
 Algorithms" by Featherstone (download from
 https://www.springer.com/us/book/9780387743141 on MIT internet)
  The tree includes an additional "rotor" body for each body.  This rotor is
 fixed to the parent body and has a gearing constraint.  This is efficiently
 included using a technique similar to what is described in Chapter 12 of
 "Robot and Multibody Dynamics" by Jain.  Note that this implementation is
 highly specific to the case of a single rotating rotor per rigid body. Rotors
 have the same joint type as their body, but with an additional gear ratio
 multiplier applied to the motion subspace. The rotors associated with the
 floating base don't do anything.
 本类存储由Featherstone在“刚体动力学算法”中描述的运动学树，该树包含一个额外的“旋转体”体。
 该旋转体固定在父体上，并具有啮合约束。
 这是有效地包括使用一种技术，类似于在Jain的“机器人和多体动力学”的第12章中所描述的。
 请注意，这种实现是高度特殊化的情况下，每个刚体只有一个旋转转子的情况
 转子有相同的关节类型作为他们的身体，但有一个附加的齿轮比率乘子适用于运动的子空间。
 与浮动基座相关的转子什么都不做。
 */

namespace sd::dynamics
{
  /*!
  * The state of a floating base model (base and joints)
  浮动基础模型的状态(基础和节点)
  */
  struct FBModelState
  {
    Quat body_orientation;
    Vector3d body_position;
    SpatialVec body_velocity; // body coordinates
    VectorXd q;
    VectorXd qd;
  };

  /*!
  * The result of running the articulated body algorithm on a rigid-body floating base model
  在刚体浮基模型上运行铰接体算法的结果
  */
  struct FBModelStateDerivative
  {
    Vector3d body_position_d;
    SpatialVec body_velocity_d;
    VectorXd qdd;
  };

  /*!
  * Class to represent a floating base rigid body model with rotors and ground
  * contacts. No concept of state.
  类来表示带转子和地面的浮动基座刚体模型联系。没有状态的概念
  */
  class FBModel
  {
  public:
    /*!
    * Initialize a floating base model with default gravity
    使用默认重力初始化浮动基模型
    */
    FBModel() : gravity_(0, 0, -9.81){};

    /*!
    * Create the floating body
    * @param inertia Spatial inertia of the floating body
    */
    void AddBase(const SpatialInertia &inertia);

    /*!
    * Create the floating body
    * @param mass Mass of the floating body
    * @param com  Center of mass of the floating body
    * @param I    Rotational inertia of the floating body
    */
    void AddBase(double mass, const Vector3d &com, const Matrix3d &I);

    /*!
    * Add a ground contact point to a model
    * @param bodyID The ID of the body containing the contact point
    * @param location The location (in body coordinate) of the contact point
    * @param isFoot True if foot or not.
    * @return The ID of the ground contact point
    */
    size_t AddGroundContactPoint(size_t body_id, const Vector3d &location,
                              bool is_foot = false);

    /*!
    * Add the bounding points of a box to the contact model. Assumes the box is
    * centered around the origin of the body coordinate system and is axis aligned.
    */
    void AddGroundContactBoxPoints(size_t body_id, const Vector3d &dims);

    /*!
    * Add a body
    * @param inertia The inertia of the body
    * @param rotor_inertia The inertia of the rotor the body is connected to
    * @param gear_ratio The gear ratio between the body and the rotor
    * @param parent The parent body, which is also assumed to be the body the rotor
    * is connected to
    * @param joint_type The type of joint (prismatic or revolute)
    * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
    * @param Xtree  The coordinate transformation from parent to this body
    * @param Xrot  The coordinate transformation from parent to this body's rotor
    * @return The body's ID (can be used as the parent)
    */
    size_t AddBody(const SpatialInertia &inertia,
                const SpatialInertia &rotor_inertia, double gear_ratio, size_t parent,
                JointType joint_type, CoordinateAxis joint_axis,
                const Matrix6d &Xtree, const Matrix6d &Xrot);

    /*!
    * Add a body
    * @param inertia The inertia of the body
    * @param rotor_inertia The inertia of the rotor the body is connected to
    * @param gear_ratio The gear ratio between the body and the rotor
    * @param parent The parent body, which is also assumed to be the body the rotor
    * is connected to
    * @param joint_type The type of joint (prismatic or revolute)
    * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
    * @param Xtree  The coordinate transformation from parent to this body
    * @param Xrot  The coordinate transformation from parent to this body's rotor
    * @return The body's ID (can be used as the parent)
    */
    size_t AddBody(const MassProperties &inertia,
                const MassProperties &rotor_inertia, double gear_ratio, size_t parent,
                JointType joint_type, CoordinateAxis joint_axis,
                const Matrix6d &Xtree, const Matrix6d &Xrot);
    void Check();

    /*!
    * Compute the total mass of bodies which are rotors
    * @return
    */
    double TotalRotorMass() const;

    /*!
    * Compute the total mass of bodies which are not rotors.
    * @return
    */
    double TotalNonRotorMass() const;

    /*!
    * Get vector of parents, where parents[i] is the parent body of body i
    其中，parents[i]是body i的父体
    * @return Vector of parents
    */
    const std::vector<size_t> &GetParentVector() const { return parents_; }

    /*!
    * Get vector of body spatial inertias
    得到机体的空间惯量矢量
    * @return Vector of body spatial inertias
    */
    const std::vector<SpatialInertia> &GetBodyInertiaVector() const
    {
      return Ibody_;
    }

    /*!
    * Get vector of rotor spatial inertias
    得到转子的空间惯量矢量
    * @return Vector of rotor spatial inertias
    */
    const std::vector<SpatialInertia> &GetRotorInertiaVector() const
    {
      return Irot_;
    }

    /*!
    * Set the gravity
    */
    void SetGravity(Vector3d &g) { gravity_ = g; }

    /*!
    * Set the flag to enable computing contact info for a given contact point
    设置此标志以启用计算给定联系点的联系信息
    * @param gc_index : index of contact point
    * @param flag : enable/disable contact calculation
    */
    void SetContactComputeFlag(size_t gc_index, bool flag)
    {
      compute_contact_info_[gc_index] = flag;
    }

    /*!
    * Compute the inverse of the contact inertia matrix (mxm)
    * @param force_ics_at_contact (3x1)
    *        e.g. if you want the cartesian inv. contact inertia in the z_ics
    *             force_ics_at_contact = [0 0 1]^T
    * @return the 1x1 inverse contact inertia J H^{-1} J^T
    */
    MatrixXd InvContactInertia(const size_t gc_index,
                               const SpatialVecXd &force_directions);

    /*!
    * Apply a unit test force at a contact. Returns the inv contact inertia  in
    * that direction and computes the resultant qdd
    * @param gc_index index of the contact
    * @param force_ics_at_contact unit test force expressed in inertial coordinates
    * @params dstate - Output paramter of resulting accelerations
    * @return the 1x1 inverse contact inertia J H^{-1} J^T
    */
    double InvContactInertia(const size_t gc_index, const Vector3d &force_ics_at_contact);

    /*!
    * Populate member variables when bodies are added
    * @param count (6 for fb, 1 for joint)
    */
    void AddDynamicsVars(size_t count);

    /*!
    * Updates the size of H, C, Cqd, G, and Js when bodies are added
    */
    void ResizeSystemMatricies();

    /*!
    * Update the state of the simulator, invalidating previous results
    更新模拟器的状态，使以前的结果无效
    * @param state : the new state
    */
    void SetState(const FBModelState &state);

    /*!
    * Mark all previously calculated values as invalid
    将所有以前计算的值标记为无效
    */
    void ResetCalculationFlags();

    /*!
    * Update the state derivative of the simulator, invalidating previous results.
    更新模拟器的状态导数，使之前的结果无效。
    * @param dstate : the new state derivative
    */
    void SetDState(const FBModelStateDerivative &dstate)
    {
      dstate_ = dstate;
      acc_uptodate_ = false;
    }

    Vector3d GetPosition(const size_t link_idx, const Vector3d &local_pos);
    Vector3d GetPosition(const size_t link_idx);

    Matrix3d GetOrientation(const size_t link_idx);
    Vector3d GetLinearVelocity(const size_t link_idx, const Vector3d &posize_t);
    Vector3d GetLinearVelocity(const size_t link_idx);

    Vector3d GetLinearAcceleration(const size_t link_idx, const Vector3d &posize_t);
    Vector3d GetLinearAcceleration(const size_t link_idx);

    Vector3d GetAngularVelocity(const size_t link_idx);
    Vector3d GetAngularAcceleration(const size_t link_idx);

    /*!
    * Forward kinematics of all bodies.  Computes Xup_ (from up the tree) and Xa_
    *(from absolute) Also computes S_ (motion subspace), _v (spatial velocity in
    *link coordinates), and c_ (coriolis acceleration in link coordinates)
    */
    void ForwardKinematics();

    /*!
    * (Support Function) Computes velocity product accelerations of
    * each link and rotor avp_, and avprot_
    */
    void BiasAccelerations();

    /*!
    * (Support Function) Computes the composite rigid body inertia
    * of each subtree IC_[i] contains body i, and the body/rotor
    * inertias of all successors of body i.
    * (key note: IC_[i] does not contain rotor i)
    */
    void CompositeInertias();
    void ForwardAccelerationKinematics();

    /*!
    * Compute the contact Jacobians (3xn matrices) for the velocity
    * of each contact point expressed in absolute coordinates
    */
    void ContactJacobians();

    // 动力学部分
    /*!
    * Computes the generalized gravitational force (G) in the inverse dynamics
    * @return G (n_dof_ x 1 vector)
    */
    const VectorXd &GeneralizedGravityForce();

    /*!
    * Computes the generalized coriolis forces (Cqd) in the inverse dynamics
    * @return Cqd (n_dof_ x 1 vector)
    */
    const VectorXd &GeneralizedCoriolisForce();

    /*!
    * Computes the Mass Matrix (H) in the inverse dynamics formulation
    * @return H (n_dof_ x n_dof_ matrix)
    */
    const MatrixXd &GeneralizedMassMatrix();

    /*!
    * Computes the inverse dynamics of the system
    * @return an n_dof_ x 1 vector. The first six entries
    * give the external wrengh on the base, with the remaining giving the
    * joint torques
    */
    VectorXd InverseDynamics(const FBModelStateDerivative &dstate);

    void RunABA(const VectorXd &tau, FBModelStateDerivative &dstate);

    /*!
    * Get the mass matrix for the system
    得到系统的质量矩阵
    */
    const MatrixXd &GetMassMatrix() const { return H_; }

    /*!
    * Get the gravity term (generalized forces)得到重力项(广义力）
    */
    const VectorXd &GetGravityForce() const { return G_; }

    /*!
    * Get the coriolis term (generalized forces) 得到科里奥利项(广义力)
    */
    const VectorXd &GetCoriolisForce() const { return Cqd_; }

    /*!
    * Support function for the ABA
    */
    void UpdateArticulatedBodies();

    /*!
    * Support function for contact inertia algorithms
    * Comptues force propagators across each joint
    */
    void UpdateForcePropagators();

    /*!
    * Support function for contact inertia algorithms
    * Computes the qdd arising from "subqdd" components
    * If you are familiar with Featherstone's sparse Op sp
    * or jain's innovations factorization:
    * H = L * D * L^T
    * These subqdd components represnt the space in the middle
    * i.e. if H^{-1} = L^{-T} * D^{-1} * L^{1}
    * then what I am calling subqdd = L^{-1} * tau
    * This is an awful explanation. It needs latex.
    */
    void UdpateQddEffects();

    /*!
    * Set all external forces to zero 设置所有外力为零
    */
    void ResetExternalForces();

  private:
    size_t n_dof_ = 0;
    Vector3d gravity_;
    std::vector<size_t> parents_;
    std::vector<double> gear_ratios_;
    std::vector<double> d_, u_;

    std::vector<JointType> joint_types_;
    std::vector<CoordinateAxis> joint_axes_;
    std::vector<Matrix6d> Xtree_, Xrot_;
    std::vector<SpatialInertia> Ibody_, Irot_;
    std::vector<std::string> body_names_;

    size_t n_ground_contact_ = 0;
    std::vector<size_t> gc_parent_;
    std::vector<Vector3d> gc_location_;
    std::vector<uint64_t> gc_foot_indices_;

    std::vector<Vector3d> gc_p_;
    std::vector<Vector3d> gc_v_;

    std::vector<bool> compute_contact_info_;

    /// BEGIN ALGORITHM SUPPORT VARIABLES 算法支持变量
    FBModelState state_;
    FBModelStateDerivative dstate_;

    std::vector<SpatialVec> v_, vrot_, a_, arot_, avp_, avprot_, c_, crot_, S_,
        Srot_, fvp_, fvprot_, ag_, agrot_, f_, frot_;

    std::vector<SpatialVec> U_, Urot_, Utot_, pA_, pArot_;
    std::vector<SpatialVec> external_forces_;

    std::vector<SpatialInertia> IC_;
    std::vector<Matrix6d> Xup_, Xa_, Xuprot_, IA_, ChiUp_;

    MatrixXd H_, C_;
    VectorXd Cqd_, G_;

    std::vector<SpatialVecXd> J_;
    std::vector<SpatialVec> Jdqd_;

    std::vector<CartesianVecXd> Jc_;
    std::vector<Vector3d> Jcdqd_;

    bool kinematics_uptodate_ = false;
    bool bias_acc_uptodate_ = false;
    bool acc_uptodate_ = false;

    bool composite_inertias_uptodate_ = false;

    bool articulated_bodies_uptodate_ = false;
    bool force_propagators_uptodate_ = false;
    bool qdd_effects_uptodate_ = false;

    MatrixXd qdd_from_base_acc_;
    MatrixXd qdd_from_subqdd_;
    Eigen::ColPivHouseholderQR<Matrix6d> invIA5_;
  };

  using FBModelPtr = std::shared_ptr<FBModel>;

} // namespace sd::dynamics
