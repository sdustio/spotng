#include "dynamics/inertia.h"
#include "math/algebra.h"

namespace sdrobot::dynamics
{
  bool BuildSpatialInertia(Eigen::Ref<SpatialInertia> ret, double const mass, Eigen::Ref<Vector3 const> const &com, Eigen::Ref<RotationalInertia const> const &inertia)
  {
    Matrix3 cSkew;
    math::VecToSkewMat(cSkew, com); //质心向量转反对称矩阵

    ret.topLeftCorner<3, 3>() =
        inertia + mass * cSkew * cSkew.transpose();
    ret.topRightCorner<3, 3>() = mass * cSkew;
    ret.bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    ret.bottomRightCorner<3, 3>() = mass * Matrix3::Identity();
    return true;
  }

  bool SpatialInertiaFlipAlongAxis(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<SpatialInertia const> const &si, CoordinateAxis const axis)
  {

    PseudoRotationalInertia P;
    SpatialInertiaToPseudoRotationalInertia(P, si);
    Matrix4 X = Matrix4::Identity();
    if (axis == CoordinateAxis::X)
      X(0, 0) = -1;
    else if (axis == CoordinateAxis::Y)
      X(1, 1) = -1;
    else if (axis == CoordinateAxis::Z)
      X(2, 2) = -1;
    P = X * P * X;
    PseudoRotationalInertiaToSpatialInertia(ret, P);
    return true;
  }

  bool SpatialInertiaToPseudoRotationalInertia(Eigen::Ref<PseudoRotationalInertia> ret, Eigen::Ref<SpatialInertia const> const &si)
  {
    Vector3 h;
    math::MatToSkewVec(h, si.topRightCorner<3, 3>());
    Matrix3 Ibar = si.topLeftCorner<3, 3>();
    double m = si(5, 5);
    ret.topLeftCorner<3, 3>() =
        0.5 * Ibar.trace() * Matrix3::Identity() - Ibar;
    ret.topRightCorner<3, 1>() = h;
    ret.bottomLeftCorner<1, 3>() = h.transpose();
    ret(3, 3) = m;
    return true;
  }

  bool PseudoRotationalInertiaToSpatialInertia(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<PseudoRotationalInertia const> const &P)
  {
    double m = P(3, 3);
    Matrix3 Ibar = P.topLeftCorner<3, 3>().trace() * Matrix3::Identity() - P.topLeftCorner<3, 3>();
    ret.topLeftCorner<3, 3>() = Ibar;
    math::VecToSkewMat(ret.topRightCorner<3, 3>(), P.topRightCorner<3, 1>());
    ret.bottomLeftCorner<3, 3>() = ret.topRightCorner<3, 3>().transpose();
    ret.bottomRightCorner<3, 3>() = m * Matrix3::Identity();
    return true;
  }

  bool BuildSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<RotMat const> const &R, Eigen::Ref<Vector3 const> const &r)
  {
    ret.setZero();
    ret.topLeftCorner<3, 3>() = R;
    ret.bottomRightCorner<3, 3>() = R;
    Matrix3 m;
    math::VecToSkewMat(m, r);
    ret.bottomLeftCorner<3, 3>() = -R * m;
    return true;
  }

  bool BuildRotationalInertia(Eigen::Ref<RotationalInertia> ret, double const mass, Eigen::Ref<Vector3 const> const &dims)
  {

    ret = Matrix3::Identity() * dims.norm() * dims.norm();
    for (int i = 0; i < 3; i++)
      ret(i, i) -= dims(i) * dims(i);
    ret = ret * mass / 12;
    return true;
  }

}
