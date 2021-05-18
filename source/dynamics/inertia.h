#pragma once

#include "dynamics/rotation.h"

namespace sd::dynamics
{
  /*!
   * Construct spatial inertia from mass, center of mass, and 3x3 rotational 从质量、质心和3 * 3的转动惯性来构造空间惯量
   * inertia
   */
  template <typename T>
  SpatialInertia<T> BuildSpatialInertia(T mass, const Vec3<T> &com, const InertiaMat<T> &inertia)
  {
    SpatialInertia<T> si;
    Mat3<T> cSkew = VecToSkewMat(com); //质心向量转反对称矩阵

    si.template topLeftCorner<3, 3>() =
        inertia + mass * cSkew * cSkew.transpose();
    si.template topRightCorner<3, 3>() = mass * cSkew;
    si.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    si.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
    return si;
  }

  template <typename T>
  SpatialInertia<T> MassPropertiesToSpatialInertia(const MassProperties<T> &a)
  {
    SpatialInertia<T> si;
    si(0, 0) = a(4);
    si(0, 1) = a(9);
    si(0, 2) = a(8);
    si(1, 0) = a(9);
    si(1, 1) = a(5);
    si(1, 2) = a(7);
    si(2, 0) = a(8);
    si(2, 1) = a(7);
    si(2, 2) = a(6);
    Mat3<T> cSkew = VecToSkewMat(Vec3<T>(a(1), a(2), a(3)));
    si.template topRightCorner<3, 3>() = cSkew;
    si.template bottomLeftCorner<3, 3>() = cSkew.transpose();
    si.template bottomRightCorner<3, 3>() = a(0) * Mat3<T>::Identity();
    return si;
  }

  /*!
   * Convert spatial inertia to mass property vector
   将空间惯性转化为质量特性矢量
   */
  template <typename T>
  MassProperties<T> SpatialInertiaToMassProperties(const SpatialInertia<T> &si)
  {
    MassProperties<T> m;
    Vec3<T> h = MatToSkewVec(si.template topRightCorner<3, 3>());
    m << si(5, 5), h(0), h(1), h(2), si(0, 0), si(1, 1),
        si(2, 2), si(2, 1), si(2, 0), si(1, 0);
    return m;
  }

  /*!
   * Get mass 得到质量
   */
  template <typename T>
  const T MassFromSpatialInertia(const SpatialInertia<T> &si) { return si(5, 5); }

  /*!
   * Get center of mass location 得到质心的位置
   */
  template <typename T>
  const Vec3<T> COMFromSpatialInertia(const SpatialInertia<T> &si)
  {
    T m = MassFromSpatialInertia(si);
    Mat3<T> mcSkew = si.template topRightCorner<3, 3>();
    return MatToSkewVec(mcSkew) / m;
  }

  /*!
   * Get 3x3 rotational inertia 得到3 * 3的转动惯量
   */
  template <typename T>
  const InertiaMat<T> SpatialInertiaToInertiaMat(const SpatialInertia<T> &si)
  {
    T m = MassFromSpatialInertia(si);
    Mat3<T> mcSkew = si.template topRightCorner<3, 3>();
    return si.template topLeftCorner<3, 3>() -
           mcSkew * mcSkew.transpose() / m;
  }

  /*!
   * Construct spatial inertia from pseudo-inertia. This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   *由伪惯性构造空间惯性。这是描述在线性矩阵不等式的物理一致的惯性参数识别:一个统计角度的质量分布，由温辛，金，斯洛廷
   */
  template <typename T>
  SpatialInertia<T> PseudoInertiaMatToSpatialInertia(const PseudoInertiaMat<T> &P)
  {
    SpatialInertia<T> si;
    T m = P(3, 3);
    Vec3<T> h = P.template topRightCorner<3, 1>();
    Mat3<T> E = P.template topLeftCorner<3, 3>();
    Mat3<T> Ibar = E.trace() * Mat3<T>::Identity() - E;
    si.template topLeftCorner<3, 3>() = Ibar;
    si.template topRightCorner<3, 3>() = VecToSkewMat(h);
    si.template bottomLeftCorner<3, 3>() = VecToSkewMat(h).transpose();
    si.template bottomRightCorner<3, 3>() = m * Mat3<T>::Identity();
    return si;
  }

  /*!
   * Convert to 4x4 pseudo-inertia matrix.  This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   */
  template <typename T>
  PseudoInertiaMat<T> SpatialInertiaToPseudoInertiaMat(const SpatialInertia<T> &si)
  {
    Vec3<T> h = MatToSkewVec(si.template topRightCorner<3, 3>());
    Mat3<T> Ibar = si.template topLeftCorner<3, 3>();
    T m = si(5, 5);
    PseudoInertiaMat<T> P;
    P.template topLeftCorner<3, 3>() =
        0.5 * Ibar.trace() * Mat3<T>::Identity() - Ibar;
    P.template topRightCorner<3, 1>() = h;
    P.template bottomLeftCorner<1, 3>() = h.transpose();
    P(3, 3) = m;
    return P;
  }

  /*!
   * Flip inertia matrix around an axis.  This isn't efficient, but it works!
   */
  template <typename T>
  SpatialInertia<T> SpatialInertiaFlipAlongAxis(const SpatialInertia<T> &si, CoordinateAxis axis)
  {
    PseudoInertiaMat<T> P = SpatialInertiaToPseudoInertiaMat(si);
    Mat4<T> X = Mat4<T>::Identity();
    if (axis == CoordinateAxis::X)
      X(0, 0) = -1;
    else if (axis == CoordinateAxis::Y)
      X(1, 1) = -1;
    else if (axis == CoordinateAxis::Z)
      X(2, 2) = -1;
    P = X * P * X;
    return SpatialInertia<T>(P);
  }

} // namespace sd::dynamics
