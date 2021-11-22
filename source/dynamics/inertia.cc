#include "dynamics/inertia.h"

#include "dynamics/rotation.h"
#include "math/algebra.h"

namespace sdquadx::dynamics {
bool BuildSpatialInertia(Eigen::Ref<SpatialInertia> ret, fpt_t const mass, Eigen::Ref<Vector3 const> const &com,
                         Eigen::Ref<InertiaTensor const> const &inertia) {
  Matrix3 cSkew;
  math::VecToSkewMat(cSkew, com);  // 质心向量转反对称矩阵

  ret.topLeftCorner<3, 3>() = inertia + mass * cSkew * cSkew.transpose();
  ret.topRightCorner<3, 3>() = mass * cSkew;
  ret.bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
  ret.bottomRightCorner<3, 3>() = mass * Matrix3::Identity();
  return true;
}

bool BuildSpatialXform(Eigen::Ref<SpatialXform> ret, CoordinateAxis const axis, fpt_t const theta) {
  RotMat R;
  CoordinateRot(R, axis, theta);
  ret.setZero();
  ret.topLeftCorner<3, 3>() = R;
  ret.bottomRightCorner<3, 3>() = R;
  return true;
}

bool SpatialInertiaFlipAlongAxis(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<SpatialInertia const> const &si,
                                 CoordinateAxis const axis) {
  PseudoInertiaTensor P;
  SpatialInertiaToPseudoInertiaTensor(P, si);
  Matrix4 X = Matrix4::Identity();
  if (axis == CoordinateAxis::X)
    X(0, 0) = -1;
  else if (axis == CoordinateAxis::Y)
    X(1, 1) = -1;
  else if (axis == CoordinateAxis::Z)
    X(2, 2) = -1;
  P = X * P * X;
  PseudoInertiaTensorToSpatialInertia(ret, P);
  return true;
}

bool SpatialInertiaToPseudoInertiaTensor(Eigen::Ref<PseudoInertiaTensor> ret,
                                         Eigen::Ref<SpatialInertia const> const &si) {
  Vector3 h;
  math::MatToSkewVec(h, si.topRightCorner<3, 3>());
  Matrix3 Ibar = si.topLeftCorner<3, 3>();
  fpt_t m = si(5, 5);
  ret.topLeftCorner<3, 3>() = 0.5 * Ibar.trace() * Matrix3::Identity() - Ibar;
  ret.topRightCorner<3, 1>() = h;
  ret.bottomLeftCorner<1, 3>() = h.transpose();
  ret(3, 3) = m;
  return true;
}

bool PseudoInertiaTensorToSpatialInertia(Eigen::Ref<SpatialInertia> ret,
                                         Eigen::Ref<PseudoInertiaTensor const> const &P) {
  fpt_t m = P(3, 3);
  Matrix3 Ibar = P.topLeftCorner<3, 3>().trace() * Matrix3::Identity() - P.topLeftCorner<3, 3>();
  ret.topLeftCorner<3, 3>() = Ibar;
  math::VecToSkewMat(ret.topRightCorner<3, 3>(), P.topRightCorner<3, 1>());
  ret.bottomLeftCorner<3, 3>() = ret.topRightCorner<3, 3>().transpose();
  ret.bottomRightCorner<3, 3>() = m * Matrix3::Identity();
  return true;
}

bool BuildSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<RotMat const> const &R,
                       Eigen::Ref<Vector3 const> const &r) {
  ret.setZero();
  ret.topLeftCorner<3, 3>() = R;
  ret.bottomRightCorner<3, 3>() = R;
  Matrix3 m;
  math::VecToSkewMat(m, r);
  ret.bottomLeftCorner<3, 3>() = -R * m;
  return true;
}

bool BuildInertiaTensor(Eigen::Ref<InertiaTensor> ret, fpt_t const mass, Eigen::Ref<Vector3 const> const &dims) {
  ret = Matrix3::Identity() * dims.norm() * dims.norm();
  for (int i = 0; i < 3; i++) ret(i, i) -= dims(i) * dims(i);
  ret = ret * mass / 12;
  return true;
}

bool MassPropertiesToSpatialInertia(Eigen::Ref<SpatialInertia> ret, Eigen::Ref<MassProperties const> const &a) {
  ret(0, 0) = a(4);
  ret(0, 1) = a(9);
  ret(0, 2) = a(8);
  ret(1, 0) = a(9);
  ret(1, 1) = a(5);
  ret(1, 2) = a(7);
  ret(2, 0) = a(8);
  ret(2, 1) = a(7);
  ret(2, 2) = a(6);
  Matrix3 cSkew;
  math::VecToSkewMat(cSkew, Vector3(a(1), a(2), a(3)));
  ret.topRightCorner<3, 3>() = cSkew;
  ret.bottomLeftCorner<3, 3>() = cSkew.transpose();
  ret.bottomRightCorner<3, 3>() = a(0) * Matrix3::Identity();
  return true;
}

bool SpatialInertiaToMassProperties(Eigen::Ref<MassProperties> ret, Eigen::Ref<SpatialInertia const> const &si) {
  Vector3 h;
  math::MatToSkewVec(h, si.topRightCorner<3, 3>());
  ret << si(5, 5), h(0), h(1), h(2), si(0, 0), si(1, 1), si(2, 2), si(2, 1), si(2, 0), si(1, 0);
  return true;
}

bool SpatialInertiaToInertiaTensor(Eigen::Ref<InertiaTensor> ret, Eigen::Ref<SpatialInertia const> const &si) {
  fpt_t m;
  MassFromSpatialInertia(m, si);
  Matrix3 mcSkew = si.topRightCorner<3, 3>();
  ret = si.topLeftCorner<3, 3>() - mcSkew * mcSkew.transpose() / m;
  return true;
}

bool MassFromSpatialInertia(fpt_t &ret, Eigen::Ref<SpatialInertia const> const &si) {
  ret = si(5, 5);
  return true;
}

bool COMFromSpatialInertia(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialInertia const> const &si) {
  fpt_t m;
  MassFromSpatialInertia(m, si);
  Matrix3 mcSkew = si.topRightCorner<3, 3>();
  math::MatToSkewVec(ret, mcSkew);
  ret = ret / m;
  return true;
}

bool SpatialXformToHomogeneous(Eigen::Ref<Matrix4> ret, Eigen::Ref<SpatialXform const> const &X) {
  ret.setZero();
  RotMat R = X.topLeftCorner<3, 3>();
  Matrix3 skewR = X.bottomLeftCorner<3, 3>();
  ret.topLeftCorner<3, 3>() = R;
  math::MatToSkewVec(ret.topRightCorner<3, 1>(), skewR * R.transpose());
  ret(3, 3) = 1;
  return true;
}

bool HomogeneousToSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<Matrix4 const> const &H) {
  Matrix3 R = H.topLeftCorner<3, 3>();
  Vector3 translate = H.topRightCorner<3, 1>();
  ret.setZero();
  ret.topLeftCorner<3, 3>() = R;
  Matrix3 m;
  math::VecToSkewMat(m, translate);
  ret.bottomLeftCorner<3, 3>() = m * R;
  ret.bottomRightCorner<3, 3>() = R;
  return true;
}

bool SpatialXformToRotMat(Eigen::Ref<RotMat> ret, Eigen::Ref<SpatialXform const> const &X) {
  ret = X.topLeftCorner<3, 3>();
  return true;
}

bool SpatialXformToTranslation(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialXform const> const &X) {
  RotMat R;
  SpatialXformToRotMat(R, X);
  math::MatToSkewVec(ret, R.transpose() * X.bottomLeftCorner<3, 3>());
  ret = -ret;
  return true;
}

bool InvertSpatialXform(Eigen::Ref<SpatialXform> ret, Eigen::Ref<SpatialXform const> const &X) {
  RotMat R;
  SpatialXformToRotMat(R, X);
  Vector3 r;
  math::MatToSkewVec(r, R.transpose() * X.bottomLeftCorner<3, 3>());
  BuildSpatialXform(ret, R.transpose(), -R * -r);
  return true;
}

bool BuildJointMotionSubspace(Eigen::Ref<SpatialVec> ret, JointType const joint, CoordinateAxis const axis) {
  Vector3 v(0, 0, 0);
  ret.setZero();
  if (axis == CoordinateAxis::X)
    v(0) = 1;
  else if (axis == CoordinateAxis::Y)
    v(1) = 1;
  else
    v(2) = 1;

  if (joint == JointType::Prismatic)
    ret.bottomLeftCorner<3, 1>() = v;
  else if (joint == JointType::Revolute)
    ret.topLeftCorner<3, 1>() = v;
  else
    throw std::runtime_error("Unknown motion subspace");

  return true;
}

bool BuildJointXform(Eigen::Ref<Matrix6> ret, JointType const joint, CoordinateAxis const axis, fpt_t const q) {
  ret.setZero();
  if (joint == JointType::Revolute) {
    BuildSpatialXform(ret, axis, q);
  } else if (joint == JointType::Prismatic) {
    Vector3 v(0, 0, 0);
    if (axis == CoordinateAxis::X)
      v(0) = q;
    else if (axis == CoordinateAxis::Y)
      v(1) = q;
    else if (axis == CoordinateAxis::Z)
      v(2) = q;

    BuildSpatialXform(ret, RotMat::Identity(), v);
  } else {
    return false;
  }
  return true;
}

bool SpatialToLinearVelocity(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &v,
                             Eigen::Ref<Vector3 const> const &x) {
  Vector3 vsAng = v.topLeftCorner<3, 1>();
  Vector3 vsLin = v.bottomLeftCorner<3, 1>();
  ret = vsLin + vsAng.cross(x);
  return true;
}

bool SpatialToAngularVelocity(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &v) {
  ret = v.topLeftCorner<3, 1>();
  return true;
}

bool SpatialToLinearAcceleration(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &a,
                                 Eigen::Ref<SpatialVec const> const &v) {
  // classical accleration = spatial linear acc + omega x v
  ret = a.tail<3>() + v.head<3>().cross(v.tail<3>());
  return true;
}

bool SpatialToLinearAcceleration(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialVec const> const &a,
                                 Eigen::Ref<SpatialVec const> const &v, Eigen::Ref<Vector3 const> const &x) {
  Vector3 alin_x;
  SpatialToLinearVelocity(alin_x, a, x);
  Vector3 vlin_x;
  SpatialToLinearVelocity(vlin_x, v, x);

  // classical accleration = spatial linear acc + omega x v
  ret = alin_x + v.head<3>().cross(vlin_x);
  return true;
}

bool SpatialXformPoint(Eigen::Ref<Vector3> ret, Eigen::Ref<SpatialXform const> const &X,
                       Eigen::Ref<Vector3 const> const &p) {
  Matrix3 R;
  SpatialXformToRotMat(R, X);
  Vector3 r;
  SpatialXformToTranslation(r, X);
  ret = R * (p - r);
  return true;
}

bool ForceToSpatialForce(Eigen::Ref<SpatialVec> ret, Eigen::Ref<Vector3 const> const &f,
                         Eigen::Ref<Vector3 const> const &p) {
  ret.topLeftCorner<3, 1>() = p.cross(f);
  ret.bottomLeftCorner<3, 1>() = f;
  return true;
}

bool MotionCrossMatrix(Eigen::Ref<Matrix6> ret, Eigen::Ref<Vector6 const> const &v) {
  ret << 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0, 0, 0, 0, 0, -v(5), v(4), 0, -v(2), v(1),
      v(5), 0, -v(3), v(2), 0, -v(0), -v(4), v(3), 0, -v(1), v(0), 0;
  return true;
}

bool ForceCrossMatrix(Eigen::Ref<Matrix6> ret, Eigen::Ref<Vector6 const> const &v) {
  ret << 0, -v(2), v(1), 0, -v(5), v(4), v(2), 0, -v(0), v(5), 0, -v(3), -v(1), v(0), 0, -v(4), v(3), 0, 0, 0, 0, 0,
      -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0;
  return true;
}

bool MotionCrossProduct(Eigen::Ref<SpatialVec> ret, Eigen::Ref<Vector6 const> const &a,
                        Eigen::Ref<Vector6 const> const &b) {
  ret << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2), a(0) * b(1) - a(1) * b(0),
      a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1), a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
      a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
  return true;
}

bool ForceCrossProduct(Eigen::Ref<SpatialVec> ret, Eigen::Ref<Vector6 const> const &a,
                       Eigen::Ref<Vector6 const> const &b) {
  ret << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4), b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
      b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3), b(5) * a(1) - b(4) * a(2), b(3) * a(2) - b(5) * a(0),
      b(4) * a(0) - b(3) * a(1);
  return true;
}

}  // namespace sdquadx::dynamics
