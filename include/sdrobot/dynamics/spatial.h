#pragma once

#include "sdrobot/dynamics/kinematics.h"

namespace sdrobot::dynamics
{
  /*!
  * Calculate the spatial coordinate transform from A to B where B is rotate by
  * theta about axis.
  */
  SpatialXform SpatialRotation(CoordinateAxis axis, double theta);

  /*!
 * Compute the spatial motion cross product matrix.
 * Prefer MotionCrossProduct when possible.
 */
  Matrix6 MotionCrossMatrix(const Vector6 &v);

  /*!
 * Compute spatial force cross product matrix.
 * Prefer ForceCrossProduct when possible
 */
  Matrix6 ForceCrossMatrix(const Vector6 &v);

  /*!
 * Compute spatial motion cross product.  Faster than the matrix multiplication
 * version
 */
  SpatialVec MotionCrossProduct(const Vector6 &a, const Vector6 &b);

  /*!
 * Compute spatial force cross product.  Faster than the matrix multiplication
 * version
 */
  SpatialVec ForceCrossProduct(const Vector6 &a, const Vector6 &b);

  /*!
 * Convert a spatial transform to a homogeneous coordinate transformation
 */
  Matrix4 SpatialXformToHomogeneous(const SpatialXform &X);

  /*!
 * Convert a homogeneous coordinate transformation to a spatial one
 */
  SpatialXform HomogeneousToSpatialXform(const Matrix4 &H);

  /*!
 * Create spatial coordinate transformation from rotation and translation
 */
  SpatialXform CreateSpatialXform(const RotMat &R, const Vector3 &r);

  /*!
 * Get rotation matrix from spatial transformation
 */
  RotMat RotationFromSpatialXform(const SpatialXform &X);

  /*!
 * Get translation vector from spatial transformation
 */
  Vector3 TranslationFromSpatialXform(const SpatialXform &X);

  /*!
 * Invert a spatial transformation (much faster than matrix inverse)
 */
  SpatialXform InvertSpatialXform(const SpatialXform &X);

  /*!
 * Compute joint motion subspace vector
 */
  SpatialVec JointMotionSubspace(JointType joint, CoordinateAxis axis);

  /*!
 * Compute joint transformation
 */
  Matrix6 JointXform(JointType joint, CoordinateAxis axis, double q);

  /*!
 * Construct the rotational inertia of a uniform density box with a given mass.
 * @param mass Mass of the box
 * @param dims Dimensions of the box
 */
  Matrix3 RotInertiaOfBox(double mass, const Vector3 &dims);

  /*!
 * Convert from spatial velocity to linear velocity.
 * Uses spatial velocity at the given point.
 */
  Vector3 SpatialToLinearVelocity(const SpatialVec &v, const Vector3 &x);

  /*!
 * Convert from spatial velocity to angular velocity.
 */
  Vector3 SpatialToAngularVelocity(const SpatialVec &v);

  /*!
 * Compute the classical lienear accleeration of a frame given its spatial
 * acceleration and velocity
 */
  Vector3 SpatialToLinearAcceleration(const SpatialVec &a, const SpatialVec &v);

  /*!
 * Compute the classical lienear acceleration of a frame given its spatial
 * acceleration and velocity
 */
  Vector3 SpatialToLinearAcceleration(const SpatialVec &a, const SpatialVec &v, const Vector3 &x);

  /*!
 * Apply spatial transformation to a point.
 */
  Vector3 SpatialXformPoint(const SpatialXform &X, const Vector3 &p);

  /*!
 * Convert a force at a point to a spatial force
 * @param f : force
 * @param p : point
 */
  SpatialVec ForceToSpatialForce(const Vector3 &f, const Vector3 &p);

}
