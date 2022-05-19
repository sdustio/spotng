#pragma once

#include "dynamics/types.h"

namespace sdengine::dynamics {
/* Build or Convert Inertias*/

/*!
 * Construct the rotational inertia of a uniform density box with a given mass.
 * @param ret
 * @param mass Mass of the box
 * @param dims Dimensions of the box
 */
bool BuildInertiaTensor(Eigen::Ref<InertiaTensor> ret, fpt_t const mass, Eigen::Ref<Vector3 const> const &dims);

/*!
 * Construct spatial inertia from mass, center of mass, and 3x3 rotational
 * inertia 从质量、质心和3 * 3的转动惯性来构造空间惯量
 */
bool BuildSpatialInertia(Eigen::Ref<SpatialInertia> ret, fpt_t const mass, Eigen::Ref<Vector3 const> const &com,
                         Eigen::Ref<InertiaTensor const> const &inertia);

/*!
 * Build spatial coordinate transformation from rotation and translation
 */
bool BuildSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<RotMat const> const &R,
                       Eigen::Ref<Vector3 const> const &r);  // CreateSpatialXform

/*!
 * Calculate the spatial coordinate transform from A to B where B is rotate by
 * theta about axis.
 */
bool BuildSpatialXform(Eigen::Ref<SpatialXform> ret, CoordinateAxis const axis, fpt_t const theta);

/*!
 * Construct spatial inertia from pseudo-inertia. This is described in
 * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
 *   Identification: A Statistical Perspective on the Mass Distribution, by
 *   Wensing, Kim, Slotine
 *由伪惯性构造空间惯性。这是描述在线性矩阵不等式的物理一致的惯性参数识别:一个统计角度的质量分布，由温辛，金，斯洛廷
 */
bool PseudoInertiaTensorToSpatialInertia(Eigen::Ref<SpatialInertia> ret,
                                         Eigen::Ref<PseudoInertiaTensor const> const &P);

/*!
 * Convert to 4x4 pseudo-inertia matrix.  This is described in
 * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
 *   Identification: A Statistical Perspective on the Mass Distribution, by
 *   Wensing, Kim, Slotine
 */
bool SpatialInertiaToPseudoInertiaTensor(Eigen::Ref<PseudoInertiaTensor> ret,
                                         Eigen::Ref<SpatialInertia const> const &si);

/*!
 * Convert mass property vector to spatial inertia
 将质量特性矢量转化为空间惯性
 */
bool MassPropertiesToSpatialInertia(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<MassProperties const> const &a);

/*!
 * Convert spatial inertia to mass property vector
 将空间惯性转化为质量特性矢量
 */
bool SpatialInertiaToMassProperties(Eigen::Ref<MassProperties> ret, Eigen::Ref<SpatialInertia const> const &si);

/*!
 * Get 3x3 rotational inertia 得到3 * 3的转动惯量
 */
bool SpatialInertiaToInertiaTensor(Eigen::Ref<InertiaTensor> ret, Eigen::Ref<SpatialInertia const> const &si);

/*!
 * Get mass 得到质量
 */
bool MassFromSpatialInertia(fpt_t &ret, Eigen::Ref<SpatialInertia const> const &si);

/*!
 * Get center of mass location 得到质心的位置
 */
bool COMFromSpatialInertia(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialInertia const> const &si);

/*!
 * Convert a spatial transform to a homogeneous coordinate transformation
 */
bool SpatialXformToHomogeneous(Eigen::Ref<Matrix4> ret, Eigen::Ref<SpatialXform const> const &X);

/*!
 * Convert a homogeneous coordinate transformation to a spatial one
 */
bool HomogeneousToSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<Matrix4 const> const &H);

/*!
 * Get rotation matrix from spatial transformation
 */
bool SpatialXformToRotMat(Eigen::Ref<RotMat> ret, Eigen::Ref<SpatialXform const> const &X);

/*!
 * Get translation vector from spatial transformation
 */
bool SpatialXformToTranslation(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialXform const> const &X);

/*!
 * Invert a spatial transformation (much faster than matrix inverse)
 */
bool InvertSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<SpatialXform const> const &X);

/*!
 * Compute joint motion subspace vector
 */
bool BuildJointMotionSubspace(Eigen::Ref<SpatialVec> ret, JointType const joint, CoordinateAxis const axis);

/*!
 * Compute joint transformation
 */
bool BuildJointXform(Eigen::Ref<Matrix6> ret, JointType const joint, CoordinateAxis const axis, fpt_t const q);

/*!
 * Convert from spatial velocity to linear velocity.
 * Uses spatial velocity at the given point.
 */
bool SpatialToLinearVelocity(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &v,
                             Eigen::Ref<Vector3 const> const &x);

/*!
 * Convert from spatial velocity to angular velocity.
 */
bool SpatialToAngularVelocity(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &v);

/*!
 * Compute the classical lienear accleeration of a frame given its spatial
 * acceleration and velocity
 */
bool SpatialToLinearAcceleration(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &a,
                                 Eigen::Ref<SpatialVec const> const &v);

/*!
 * Compute the classical lienear acceleration of a frame given its spatial
 * acceleration and velocity
 */
bool SpatialToLinearAcceleration(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &a,
                                 Eigen::Ref<SpatialVec const> const &v, Eigen::Ref<Vector3 const> const &x);

/*!
 * Apply spatial transformation to a point.
 */
bool SpatialXformPoint(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialXform const> const &X,
                       Eigen::Ref<Vector3 const> const &p);

/*!
 * Convert a force at a point to a spatial force
 * @param f : force
 * @param p : point
 */
bool ForceToSpatialForce(Eigen::Ref<SpatialVec> ret, Eigen::Ref<Vector3 const> const &f,
                         Eigen::Ref<Vector3 const> const &p);

/* Operations and Calculates */

/*!
 * Flip inertia matrix around an axis.  This isn't efficient, but it works!
 */
bool SpatialInertiaFlipAlongAxis(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<SpatialInertia const> const &si,
                                 CoordinateAxis const axis);

/*!
 * Compute the spatial motion cross product matrix.
 * Prefer MotionCrossProduct when possible.
 */
bool MotionCrossMatrix(Eigen::Ref<Matrix6> ret, Eigen::Ref<SpatialVec const> const &v);

/*!
 * Compute spatial force cross product matrix.
 * Prefer ForceCrossProduct when possible
 */
bool ForceCrossMatrix(Eigen::Ref<Matrix6> ret, Eigen::Ref<SpatialVec const> const &v);

/*!
 * Compute spatial motion cross product.  Faster than the matrix multiplication
 * version
 */
bool MotionCrossProduct(Eigen::Ref<SpatialVec> ret, Eigen::Ref<SpatialVec const> const &a,
                        Eigen::Ref<SpatialVec const> const &b);

/*!
 * Compute spatial force cross product.  Faster than the matrix multiplication
 * version
 */
bool ForceCrossProduct(Eigen::Ref<SpatialVec> ret, Eigen::Ref<SpatialVec const> const &a,
                       Eigen::Ref<SpatialVec const> const &b);

}  // namespace sdengine::dynamics
