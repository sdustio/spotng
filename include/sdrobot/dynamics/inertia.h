#pragma once

#include "sdrobot/dynamics/kinematics.h"

namespace sdrobot::dynamics
{
  /*!
   * Construct spatial inertia from mass, center of mass, and 3x3 rotational 从质量、质心和3 * 3的转动惯性来构造空间惯量
   * inertia
   */
  SpatialInertia BuildSpatialInertia(double mass, const Vector3d &com, const InertiaMat &inertia);

  SpatialInertia MassPropertiesToSpatialInertia(const MassProperties &a);

  /*!
   * Convert spatial inertia to mass property vector
   将空间惯性转化为质量特性矢量
   */
  MassProperties SpatialInertiaToMassProperties(const SpatialInertia &si);

  /*!
   * Get mass 得到质量
   */
  double MassFromSpatialInertia(const SpatialInertia &si);

  /*!
   * Get center of mass location 得到质心的位置
   */
  Vector3d COMFromSpatialInertia(const SpatialInertia &si);

  /*!
   * Get 3x3 rotational inertia 得到3 * 3的转动惯量
   */
  InertiaMat SpatialInertiaToInertiaMat(const SpatialInertia &si);

  /*!
   * Construct spatial inertia from pseudo-inertia. This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   *由伪惯性构造空间惯性。这是描述在线性矩阵不等式的物理一致的惯性参数识别:一个统计角度的质量分布，由温辛，金，斯洛廷
   */
  SpatialInertia PseudoInertiaMatToSpatialInertia(const PseudoInertiaMat &P);

  /*!
   * Convert to 4x4 pseudo-inertia matrix.  This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   */
  PseudoInertiaMat SpatialInertiaToPseudoInertiaMat(const SpatialInertia &si);

  /*!
   * Flip inertia matrix around an axis.  This isn't efficient, but it works!
   */
  SpatialInertia SpatialInertiaFlipAlongAxis(const SpatialInertia &si, CoordinateAxis axis);

} // namespace sdrobot::dynamics
