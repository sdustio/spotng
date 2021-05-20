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

#pragma once

#include <vector>

#include "dynamics/inertia.h"

namespace sd::dynamics
{
  /*!
  * The state of a floating base model (base and joints)
  浮动基础模型的状态(基础和节点)
  */
  template <typename T>
  struct FBModelState
  {
    Quat<T> body_orientation;
    Vec3<T> body_position;
    SVec<T> body_velocity; // body coordinates
    DVec<T> q;
    DVec<T> qd;
  };

  /*!
  * The result of running the articulated body algorithm on a rigid-body floating base model
  在刚体浮基模型上运行铰接体算法的结果
  */
  template <typename T>
  struct FBModelStateDerivative
  {
    Vec3<T> body_position_d;
    SVec<T> body_velocity_d;
    DVec<T> qdd;
  };

  /*!
  * Class to represent a floating base rigid body model with rotors and ground
  * contacts. No concept of state.
  类来表示带转子和地面的浮动基座刚体模型联系。没有状态的概念
  */
  template <typename T>
  class FBModel
  {
  public:
    /*!
    * Initialize a floating base model with default gravity
    使用默认重力初始化浮动基模型
    */
    FBModel() : gravity_(0, 0, -9.81){};

    void AddBase(const SpatialInertia<T> &inertia);
    void AddBase(T mass, const Vec3<T> &com, const Mat3<T> &I);
    int AddGroundContactPoint(int bodyID, const Vec3<T> &location,
                              bool isFoot = false);
    void AddGroundContactBoxPoints(int bodyId, const Vec3<T> &dims);
    int AddBody(const SpatialInertia<T> &inertia,
                const SpatialInertia<T> &rotorInertia, T gearRatio, int parent,
                JointType jointType, CoordinateAxis jointAxis,
                const Mat6<T> &Xtree, const Mat6<T> &Xrot);
    int AddBody(const MassProperties<T> &inertia,
                const MassProperties<T> &rotorInertia, T gearRatio, int parent,
                JointType jointType, CoordinateAxis jointAxis,
                const Mat6<T> &Xtree, const Mat6<T> &Xrot);
    void Check();
    T TotalRotorMass();
    T TotalNonRotorMass();

    /*!
    * Get vector of parents, where parents[i] is the parent body of body i
    其中，parents[i]是body i的父体
    * @return Vector of parents
    */
    const std::vector<int> &GetParentVector() { return parents_; }

    /*!
    * Get vector of body spatial inertias
    得到机体的空间惯量矢量
    * @return Vector of body spatial inertias
    */
    const std::vector<SpatialInertia<T>> &GetBodyInertiaVector()
    {
      return Ibody_;
    }

    /*!
    * Get vector of rotor spatial inertias
    得到转子的空间惯量矢量
    * @return Vector of rotor spatial inertias
    */
    const std::vector<SpatialInertia<T>> &GetRotorInertiaVector()
    {
      return Irot_;
    }

    /*!
    * Set the gravity
    */
    void SetGravity(Vec3<T> &g) { gravity_ = g; }

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

    DMat<T> InvContactInertia(const int gc_index,
                              const D6Mat<T> &force_directions);
    T InvContactInertia(const int gc_index, const Vec3<T> &force_ics_at_contact);

    T ApplyTestForce(const int gc_index, const Vec3<T> &force_ics_at_contact,
                     FBModelStateDerivative<T> &dstate_out);

    T ApplyTestForce(const int gc_index, const Vec3<T> &force_ics_at_contact,
                     DVec<T> &dstate_out);

    void AddDynamicsVars(int count);

    void ResizeSystemMatricies();

    /*!
    * Update the state of the simulator, invalidating previous results
    更新模拟器的状态，使以前的结果无效
    * @param state : the new state
    */
    void SetState(const FBModelState<T> &state)
    {
      state_ = state;

      bias_acc_uptodate_ = false;
      composite_inertias_uptodate_ = false;

      ResetCalculationFlags();
    }

    /*!
    * Mark all previously calculated values as invalid
    将所有以前计算的值标记为无效
    */
    void ResetCalculationFlags()
    {
      articulated_bodies_uptodate_ = false;
      kinematics_uptodate_ = false;
      force_propagators_uptodate_ = false;
      qdd_effects_uptodate_ = false;
      acc_uptodate_ = false;
    }

    /*!
    * Update the state derivative of the simulator, invalidating previous results.
    更新模拟器的状态导数，使之前的结果无效。
    * @param dState : the new state derivative
    */
    void SetDState(const FBModelStateDerivative<T> &dState)
    {
      dstate_ = dState;
      acc_uptodate_ = false;
    }

    Vec3<T> GetPosition(const int link_idx, const Vec3<T> &local_pos);
    Vec3<T> GetPosition(const int link_idx);

    Mat3<T> GetOrientation(const int link_idx);
    Vec3<T> GetLinearVelocity(const int link_idx, const Vec3<T> &point);
    Vec3<T> GetLinearVelocity(const int link_idx);

    Vec3<T> GetLinearAcceleration(const int link_idx, const Vec3<T> &point);
    Vec3<T> GetLinearAcceleration(const int link_idx);

    Vec3<T> GetAngularVelocity(const int link_idx);
    Vec3<T> GetAngularAcceleration(const int link_idx);

    void ForwardKinematics();
    void BiasAccelerations();
    void CompositeInertias();
    void ForwardAccelerationKinematics();
    void ContactJacobians();

    // 动力学部分
    DVec<T> GeneralizedGravityForce();
    DVec<T> GeneralizedCoriolisForce();
    DMat<T> GeneralizedMassMatrix();

    DVec<T> InverseDynamics(const FBModelStateDerivative<T> &dState);
    void RunABA(const DVec<T> &tau, FBModelStateDerivative<T> &dstate);

    /*!
    * Get the mass matrix for the system
    得到系统的质量矩阵
    */
    const DMat<T> &GetMassMatrix() const { return H_; }

    /*!
    * Get the gravity term (generalized forces)得到重力项(广义力）
    */
    const DVec<T> &GetGravityForce() const { return G_; }

    /*!
    * Get the coriolis term (generalized forces) 得到科里奥利项(广义力)
    */
    const DVec<T> &GetCoriolisForce() const { return Cqd_; }

    void UpdateArticulatedBodies();
    void UpdateForcePropagators();
    void UdpateQddEffects();

    /*!
    * Set all external forces to zero 设置所有外力为零
    */
    void ResetExternalForces()
    {
      for (size_t i = 0; i < n_dof_; i++)
      {
        external_forces_[i] = SVec<T>::Zero();
      }
    }

  private:
    size_t n_dof_ = 0;
    Vec3<T> gravity_;
    std::vector<int> parents_;
    std::vector<T> gear_ratios_;
    std::vector<T> d_, u_;

    std::vector<JointType> joint_types_;
    std::vector<CoordinateAxis> joint_axes_;
    std::vector<Mat6<T>> Xtree_, Xrot_;
    std::vector<SpatialInertia<T>> Ibody_, Irot_;
    std::vector<std::string> body_names_;

    size_t n_ground_contact_ = 0;
    std::vector<size_t> gc_parent_;
    std::vector<Vec3<T>> gc_location_;
    std::vector<uint64_t> gc_foot_indices_;

    std::vector<Vec3<T>> gc_p_;
    std::vector<Vec3<T>> gc_v_;

    std::vector<bool> compute_contact_info_;

    /// BEGIN ALGORITHM SUPPORT VARIABLES 算法支持变量
    FBModelState<T> state_;
    FBModelStateDerivative<T> dstate_;

    std::vector<SVec<T>> v_, vrot_, a_, arot_, avp_, avprot_, c_, crot_, S_,
        Srot_, fvp_, fvprot_, ag_, agrot_, f_, frot_;

    std::vector<SVec<T>> U_, Urot_, Utot_, pA_, pArot_;
    std::vector<SVec<T>> external_forces_;

    std::vector<SpatialInertia<T>> IC_;
    std::vector<Mat6<T>> Xup_, Xa_, Xuprot_, IA_, ChiUp_;

    DMat<T> H_, C_;
    DVec<T> Cqd_, G_;

    std::vector<D6Mat<T>> J_;
    std::vector<SVec<T>> Jdqd_;

    std::vector<D3Mat<T>> Jc_;
    std::vector<Vec3<T>> Jcdqd_;

    bool kinematics_uptodate_ = false;
    bool bias_acc_uptodate_ = false;
    bool acc_uptodate_ = false;

    bool composite_inertias_uptodate_ = false;

    bool articulated_bodies_uptodate_ = false;
    bool force_propagators_uptodate_ = false;
    bool qdd_effects_uptodate_ = false;

    DMat<T> qdd_from_base_acc_;
    DMat<T> qdd_from_subqdd_;
    Eigen::ColPivHouseholderQR<Mat6<T>> invIA5_;
  };

} // namespace sd::dynamics
