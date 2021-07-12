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
  Matrix6d MotionCrossMatrix(const Vector6d &v);

  /*!
 * Compute spatial force cross product matrix.
 * Prefer ForceCrossProduct when possible
 */
  Matrix6d ForceCrossMatrix(const Vector6d &v);

  /*!
 * Compute spatial motion cross product.  Faster than the matrix multiplication
 * version
 */
  SpatialVec MotionCrossProduct(const Vector6d &a, const Vector6d &b);

  /*!
 * Compute spatial force cross product.  Faster than the matrix multiplication
 * version
 */
  SpatialVec ForceCrossProduct(const Vector6d &a, const Vector6d &b);

  /*!
 * Convert a spatial transform to a homogeneous coordinate transformation
 */
  Matrix4d SpatialXformToHomogeneous(const SpatialXform &X);

  /*!
 * Convert a homogeneous coordinate transformation to a spatial one
 */
  SpatialXform HomogeneousToSpatialXform(const Matrix4d &H);

  /*!
 * Create spatial coordinate transformation from rotation and translation
 */
  SpatialXform CreateSpatialXform(const RotMat &R, const Vector3d &r);

  /*!
 * Get rotation matrix from spatial transformation
 */
  RotMat RotationFromSpatialXform(const SpatialXform &X);

  /*!
 * Get translation vector from spatial transformation
 */
  Vector3d TranslationFromSpatialXform(const SpatialXform &X);

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
  Matrix6d JointXform(JointType joint, CoordinateAxis axis, double q);

  /*!
 * Construct the rotational inertia of a uniform density box with a given mass.
 * @param mass Mass of the box
 * @param dims Dimensions of the box
 */
  Matrix3d RotInertiaOfBox(double mass, const Vector3d &dims);

  /*!
 * Convert from spatial velocity to linear velocity.
 * Uses spatial velocity at the given point.
 */
  Vector3d SpatialToLinearVelocity(const SpatialVec &v, const Vector3d &x);

  /*!
 * Convert from spatial velocity to angular velocity.
 */
  Vector3d SpatialToAngularVelocity(const SpatialVec &v);

  /*!
 * Compute the classical lienear accleeration of a frame given its spatial
 * acceleration and velocity
 */
  Vector3d SpatialToLinearAcceleration(const SpatialVec &a, const SpatialVec &v);

  /*!
 * Compute the classical lienear acceleration of a frame given its spatial
 * acceleration and velocity
 */
  Vector3d SpatialToLinearAcceleration(const SpatialVec &a, const SpatialVec &v, const Vector3d &x);

  /*!
 * Apply spatial transformation to a point.
 */
  Vector3d SpatialXformPoint(const SpatialXform &X, const Vector3d &p);

  /*!
 * Convert a force at a point to a spatial force
 * @param f : force
 * @param p : point
 */
  SpatialVec ForceToSpatialForce(const Vector3d &f, const Vector3d &p);

}
