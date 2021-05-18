#pragma once

#include "dynamics/inertia.h"

namespace sd::dynamics
{
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

  /*!
 * The state of a floating base model (base and joints)
 浮动基础模型的状态(基础和节点)
 */
  template <typename T>
  struct FBModelState
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quat<T> bodyOrientation;
    Vec3<T> bodyPosition;
    SVec<T> bodyVelocity; // body coordinates
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<T> dBodyPosition;
    SVec<T> dBodyVelocity;
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
    FBModel() : mGravity(0, 0, -9.81) {};

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
    T totalRotorMass();
    T totalNonRotorMass();

    /*!
   * Get vector of parents, where parents[i] is the parent body of body i
   其中，parents[i]是body i的父体
   * @return Vector of parents
   */
    const std::vector<int> &getParentVector() { return _parents; }

    /*!
   * Get vector of body spatial inertias
   得到机体的空间惯量矢量
   * @return Vector of body spatial inertias
   */
    const std::vector<SpatialInertia<T>,
                      Eigen::aligned_allocator<SpatialInertia<T>>> &
    getBodyInertiaVector()
    {
      return _Ibody;
    }

    /*!
   * Get vector of rotor spatial inertias
   得到转子的空间惯量矢量
   * @return Vector of rotor spatial inertias
   */
    const std::vector<SpatialInertia<T>,
                      Eigen::aligned_allocator<SpatialInertia<T>>> &
    getRotorInertiaVector()
    {
      return _Irot;
    }

    /*!
   * Set the gravity
   */
    void setGravity(Vec3<T> &g) { mGravity = g; }

    /*!
   * Set the flag to enable computing contact info for a given contact point
   设置此标志以启用计算给定联系点的联系信息
   * @param gc_index : index of contact point
   * @param flag : enable/disable contact calculation
   */
    void setContactComputeFlag(size_t gc_index, bool flag)
    {
      _compute_contact_info[gc_index] = flag;
    }

    DMat<T> invContactInertia(const int gc_index,
                              const D6Mat<T> &force_directions);
    T invContactInertia(const int gc_index, const Vec3<T> &force_ics_at_contact);

    T applyTestForce(const int gc_index, const Vec3<T> &force_ics_at_contact,
                     FBModelStateDerivative<T> &dstate_out);

    T applyTestForce(const int gc_index, const Vec3<T> &force_ics_at_contact,
                     DVec<T> &dstate_out);

    void addDynamicsVars(int count);

    void resizeSystemMatricies();

    /*!
   * Update the state of the simulator, invalidating previous results
   更新模拟器的状态，使以前的结果无效
   * @param state : the new state
   */
    void setState(const FBModelState<T> &state)
    {
      _state = state;

      _biasAccelerationsUpToDate = false;
      _compositeInertiasUpToDate = false;

      resetCalculationFlags();
    }

    /*!
   * Mark all previously calculated values as invalid
   将所有以前计算的值标记为无效
   */
    void resetCalculationFlags()
    {
      _articulatedBodiesUpToDate = false;
      _kinematicsUpToDate = false;
      _forcePropagatorsUpToDate = false;
      _qddEffectsUpToDate = false;
      _accelerationsUpToDate = false;
    }

    /*!
   * Update the state derivative of the simulator, invalidating previous results.
   更新模拟器的状态导数，使之前的结果无效。
   * @param dState : the new state derivative
   */
    void setDState(const FBModelStateDerivative<T> &dState)
    {
      _dState = dState;
      _accelerationsUpToDate = false;
    }

    Vec3<T> getPosition(const int link_idx, const Vec3<T> &local_pos);
    Vec3<T> getPosition(const int link_idx);

    Mat3<T> getOrientation(const int link_idx);
    Vec3<T> getLinearVelocity(const int link_idx, const Vec3<T> &point);
    Vec3<T> getLinearVelocity(const int link_idx);

    Vec3<T> getLinearAcceleration(const int link_idx, const Vec3<T> &point);
    Vec3<T> getLinearAcceleration(const int link_idx);

    Vec3<T> getAngularVelocity(const int link_idx);
    Vec3<T> getAngularAcceleration(const int link_idx);

    void forwardKinematics();
    void biasAccelerations();
    void compositeInertias();
    void forwardAccelerationKinematics();
    void contactJacobians();

    DVec<T> generalizedGravityForce();
    DVec<T> generalizedCoriolisForce();
    DMat<T> massMatrix();
    DVec<T> inverseDynamics(const FBModelStateDerivative<T> &dState);
    void runABA(const DVec<T> &tau, FBModelStateDerivative<T> &dstate);

    size_t _nDof = 0;
    Vec3<T> mGravity;
    vector<int> _parents;
    vector<T> _gearRatios;
    vector<T> _d, _u;

    vector<JointType> _jointTypes;
    vector<CoordinateAxis> _jointAxes;
    vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
    vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>> _Ibody,
        _Irot;
    vector<std::string> _bodyNames;

    size_t _nGroundContact = 0;
    vector<size_t> _gcParent;
    vector<Vec3<T>> _gcLocation;
    vector<uint64_t> _footIndicesGC;

    vector<Vec3<T>> _pGC;
    vector<Vec3<T>> _vGC;

    vector<bool> _compute_contact_info;

    /*!
  动力学部分
   * Get the mass matrix for the system
   得到系统的质量矩阵
   */
    const DMat<T> &getMassMatrix() const { return _H; }

    /*!
   * Get the gravity term (generalized forces)得到重力项(广义力）
   */
    const DVec<T> &getGravityForce() const { return _G; }

    /*!
   * Get the coriolis term (generalized forces) 得到科里奥利项(广义力)
   */
    const DVec<T> &getCoriolisForce() const { return _Cqd; }

    /// BEGIN ALGORITHM SUPPORT VARIABLES 算法支持变量
    FBModelState<T> _state;
    FBModelStateDerivative<T> _dState;

    vectorAligned<SVec<T>> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S,
        _Srot, _fvp, _fvprot, _ag, _agrot, _f, _frot;

    vectorAligned<SVec<T>> _U, _Urot, _Utot, _pA, _pArot;
    vectorAligned<SVec<T>> _externalForces;

    vectorAligned<SpatialInertia<T>> _IC;
    vectorAligned<Mat6<T>> _Xup, _Xa, _Xuprot, _IA, _ChiUp;

    DMat<T> _H, _C;
    DVec<T> _Cqd, _G;

    vectorAligned<D6Mat<T>> _J;
    vectorAligned<SVec<T>> _Jdqd;

    vectorAligned<D3Mat<T>> _Jc;
    vectorAligned<Vec3<T>> _Jcdqd;

    bool _kinematicsUpToDate = false;
    bool _biasAccelerationsUpToDate = false;
    bool _accelerationsUpToDate = false;

    bool _compositeInertiasUpToDate = false;

    void updateArticulatedBodies();
    void updateForcePropagators();
    void udpateQddEffects();

    /*!
   * Set all external forces to zero 设置所有外力为零
   */
    void resetExternalForces()
    {
      for (size_t i = 0; i < _nDof; i++)
      {
        _externalForces[i] = SVec<T>::Zero();
      }
    }

    bool _articulatedBodiesUpToDate = false;
    bool _forcePropagatorsUpToDate = false;
    bool _qddEffectsUpToDate = false;

    DMat<T> _qdd_from_base_accel;
    DMat<T> _qdd_from_subqdd;
    Eigen::ColPivHouseholderQR<Mat6<T>> _invIA5;
  };

} // namespace sd::dynamics
