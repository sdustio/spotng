#include "sdrobot/dynamics/rotation.h"
#include "sdrobot/dynamics/inertia.h"

namespace sdrobot::dynamics
{
  /*!
   * Construct spatial inertia from mass, center of mass, and 3x3 rotational 从质量、质心和3 * 3的转动惯性来构造空间惯量
   * inertia
   */
  SpatialInertia BuildSpatialInertia(double mass, const Vector3 &com, const InertiaMat &inertia)
  {
    SpatialInertia si;
    Matrix3 cSkew = VecToSkewMat(com); //质心向量转反对称矩阵

    si.topLeftCorner<3, 3>() =
        inertia + mass * cSkew * cSkew.transpose();
    si.topRightCorner<3, 3>() = mass * cSkew;
    si.bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    si.bottomRightCorner<3, 3>() = mass * Matrix3::Identity();
    return si;
  }

  SpatialInertia MassPropertiesToSpatialInertia(const MassProperties &a)
  {
    SpatialInertia si;
    si(0, 0) = a(4);
    si(0, 1) = a(9);
    si(0, 2) = a(8);
    si(1, 0) = a(9);
    si(1, 1) = a(5);
    si(1, 2) = a(7);
    si(2, 0) = a(8);
    si(2, 1) = a(7);
    si(2, 2) = a(6);
    Matrix3 cSkew = VecToSkewMat(Vector3(a(1), a(2), a(3)));
    si.topRightCorner<3, 3>() = cSkew;
    si.bottomLeftCorner<3, 3>() = cSkew.transpose();
    si.bottomRightCorner<3, 3>() = a(0) * Matrix3::Identity();
    return si;
  }

  /*!
   * Convert spatial inertia to mass property vector
   将空间惯性转化为质量特性矢量
   */
  MassProperties SpatialInertiaToMassProperties(const SpatialInertia &si)
  {
    MassProperties m;
    Vector3 h = MatToSkewVec(si.topRightCorner<3, 3>());
    m << si(5, 5), h(0), h(1), h(2), si(0, 0), si(1, 1),
        si(2, 2), si(2, 1), si(2, 0), si(1, 0);
    return m;
  }

  /*!
   * Get mass 得到质量
   */
  double MassFromSpatialInertia(const SpatialInertia &si) { return si(5, 5); }

  /*!
   * Get center of mass location 得到质心的位置
   */
  Vector3 COMFromSpatialInertia(const SpatialInertia &si)
  {
    double m = MassFromSpatialInertia(si);
    Matrix3 mcSkew = si.topRightCorner<3, 3>();
    return MatToSkewVec(mcSkew) / m;
  }

  /*!
   * Get 3x3 rotational inertia 得到3 * 3的转动惯量
   */
  InertiaMat SpatialInertiaToInertiaMat(const SpatialInertia &si)
  {
    double m = MassFromSpatialInertia(si);
    Matrix3 mcSkew = si.topRightCorner<3, 3>();
    return si.topLeftCorner<3, 3>() -
           mcSkew * mcSkew.transpose() / m;
  }

  /*!
   * Construct spatial inertia from pseudo-inertia. This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   *由伪惯性构造空间惯性。这是描述在线性矩阵不等式的物理一致的惯性参数识别:一个统计角度的质量分布，由温辛，金，斯洛廷
   */
  SpatialInertia PseudoInertiaMatToSpatialInertia(const PseudoInertiaMat &P)
  {
    SpatialInertia si;
    double m = P(3, 3);
    Vector3 h = P.topRightCorner<3, 1>();
    Matrix3 E = P.topLeftCorner<3, 3>();
    Matrix3 Ibar = E.trace() * Matrix3::Identity() - E;
    si.topLeftCorner<3, 3>() = Ibar;
    si.topRightCorner<3, 3>() = VecToSkewMat(h);
    si.bottomLeftCorner<3, 3>() = VecToSkewMat(h).transpose();
    si.bottomRightCorner<3, 3>() = m * Matrix3::Identity();
    return si;
  }

  /*!
   * Convert to 4x4 pseudo-inertia matrix.  This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   */
  PseudoInertiaMat SpatialInertiaToPseudoInertiaMat(const SpatialInertia &si)
  {
    Vector3 h = MatToSkewVec(si.topRightCorner<3, 3>());
    Matrix3 Ibar = si.topLeftCorner<3, 3>();
    double m = si(5, 5);
    PseudoInertiaMat P;
    P.topLeftCorner<3, 3>() =
        0.5 * Ibar.trace() * Matrix3::Identity() - Ibar;
    P.topRightCorner<3, 1>() = h;
    P.bottomLeftCorner<1, 3>() = h.transpose();
    P(3, 3) = m;
    return P;
  }

  /*!
   * Flip inertia matrix around an axis.  This isn't efficient, but it works!
   */
  SpatialInertia SpatialInertiaFlipAlongAxis(const SpatialInertia &si, CoordinateAxis axis)
  {
    PseudoInertiaMat P = SpatialInertiaToPseudoInertiaMat(si);
    Matrix4 X = Matrix4::Identity();
    if (axis == CoordinateAxis::X)
      X(0, 0) = -1;
    else if (axis == CoordinateAxis::Y)
      X(1, 1) = -1;
    else if (axis == CoordinateAxis::Z)
      X(2, 2) = -1;
    P = X * P * X;
    return PseudoInertiaMatToSpatialInertia(P);
  }

} // namespace sdrobot::dynamics
