#pragma once

#include "sdrobot/dynamics.h"
#include "dynamics/types.h"

namespace sdrobot::dynamics
{
  /*!
   * Construct spatial inertia from mass, center of mass, and 3x3 rotational inertia
   * 从质量、质心和3 * 3的转动惯性来构造空间惯量
   */
  bool BuildSpatialInertia(Eigen::Ref<SpatialInertia> ret, double const mass, Eigen::Ref<Vector3 const> const &com, Eigen::Ref<RotationalInertia const> const &inertia);

  /*!
   * Construct spatial inertia from pseudo-inertia. This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   *由伪惯性构造空间惯性。这是描述在线性矩阵不等式的物理一致的惯性参数识别:一个统计角度的质量分布，由温辛，金，斯洛廷
   */
  bool PseudoRotationalInertiaToSpatialInertia(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<PseudoRotationalInertia const> const &P);

  /*!
   * Convert to 4x4 pseudo-inertia matrix.  This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   */
  bool SpatialInertiaToPseudoRotationalInertia(Eigen::Ref<PseudoRotationalInertia> ret, Eigen::Ref<SpatialInertia const> const &si);

  /*!
  * Flip inertia matrix around an axis.  This isn't efficient, but it works!
  */
  bool SpatialInertiaFlipAlongAxis(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<SpatialInertia const> const &si, CoordinateAxis const axis);

  /*!
  * Build spatial coordinate transformation from rotation and translation
  */
  bool BuildSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<RotMat const> const &R, Eigen::Ref<Vector3 const> const &r);

  /*!
  * Construct the rotational inertia of a uniform density box with a given mass.
  * @param ret
  * @param mass Mass of the box
  * @param dims Dimensions of the box
  */
  bool BuildRotationalInertia(Eigen::Ref<RotationalInertia> ret, double const mass, Eigen::Ref<Vector3 const> const &dims);
}
