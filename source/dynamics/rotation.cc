#include "dynamics/rotation.h"

#include <algorithm>
#include <cmath>

#include "math/utils.h"

namespace sdquadx::dynamics {
/*!
 * Compute rotation matrix for coordinate transformation. Note that
 * CoordinateRot(CoordinateAxis:X, .1) * v will rotate v by .1 radians -
 * this transforms into a frame rotated by .1 radians!.
 * 计算坐标变换的旋转矩阵。请注意,
 * 坐标(坐标轴:X， .1) * v将v旋转 .1弧度-
 * 转换成一个旋转了1弧度的帧!
 */
bool CoordinateRot(Eigen::Ref<RotMat> ret, CoordinateAxis const axis, fpt_t const theta) {
  fpt_t s = std::sin(theta);
  fpt_t c = std::cos(theta);

  if (axis == CoordinateAxis::X) {
    ret << 1, 0, 0, 0, c, -s, 0, s, c;
  } else if (axis == CoordinateAxis::Y) {
    ret << c, 0, s, 0, 1, 0, -s, 0, c;
  } else if (axis == CoordinateAxis::Z) {
    ret << c, -s, 0, s, c, 0, 0, 0, 1;
  }
  return true;
}

/*!
 * Go from rpy to rotation matrix.从欧拉角转矩阵
 */
bool RPYToRotMat(Eigen::Ref<RotMat> ret, Eigen::Ref<Vector3 const> const &v) {
  CoordinateRot(ret, CoordinateAxis::Z, v[2]);
  RotMat tmp;
  CoordinateRot(tmp, CoordinateAxis::Y, v[1]);
  ret = ret * tmp;
  CoordinateRot(tmp, CoordinateAxis::X, v[0]);
  ret = ret * tmp;
  return true;
}

/*!
 * Convert a coordinate transformation matrix to an orientation quaternion.
 * 将坐标变换矩阵转换为方向四元数
 */
bool RotMatToQuat(Eigen::Ref<Quat> ret, Eigen::Ref<RotMat const> const &r1) {
  fpt_t tr = r1.trace();
  if (tr > 0.0) {
    fpt_t S = sqrt(tr + 1.0) * 2.0;
    ret(0) = 0.25 * S;
    ret(1) = (r1(2, 1) - r1(1, 2)) / S;
    ret(2) = (r1(0, 2) - r1(2, 0)) / S;
    ret(3) = (r1(1, 0) - r1(0, 1)) / S;
  } else if ((r1(0, 0) > r1(1, 1)) && (r1(0, 0) > r1(2, 2))) {
    fpt_t S = sqrt(1.0 + r1(0, 0) - r1(1, 1) - r1(2, 2)) * 2.0;
    ret(0) = (r1(2, 1) - r1(1, 2)) / S;
    ret(1) = 0.25 * S;
    ret(2) = (r1(0, 1) + r1(1, 0)) / S;
    ret(3) = (r1(0, 2) + r1(2, 0)) / S;
  } else if (r1(1, 1) > r1(2, 2)) {
    fpt_t S = sqrt(1.0 + r1(1, 1) - r1(0, 0) - r1(2, 2)) * 2.0;
    ret(0) = (r1(0, 2) - r1(2, 0)) / S;
    ret(1) = (r1(0, 1) + r1(1, 0)) / S;
    ret(2) = 0.25 * S;
    ret(3) = (r1(1, 2) + r1(2, 1)) / S;
  } else {
    fpt_t S = sqrt(1.0 + r1(2, 2) - r1(0, 0) - r1(1, 1)) * 2.0;
    ret(0) = (r1(1, 0) - r1(0, 1)) / S;
    ret(1) = (r1(0, 2) + r1(2, 0)) / S;
    ret(2) = (r1(1, 2) + r1(2, 1)) / S;
    ret(3) = 0.25 * S;
  }
  return true;
}

/*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 *将四元数转换为旋转矩阵。这个矩阵表示
 *坐标转换为指定方向的具有指定方向的坐标系 通过四元数
 */
bool QuatToRotMat(Eigen::Ref<RotMat> ret, Eigen::Ref<Quat const> const &q) {
  fpt_t e0 = q(0);
  fpt_t e1 = q(1);
  fpt_t e2 = q(2);
  fpt_t e3 = q(3);

  ret << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1), 2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
  return true;
}

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
bool QuatToRPY(Eigen::Ref<Vector3> ret, Eigen::Ref<Quat const> const &q) {
  fpt_t as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  ret(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                      math::Square(q[0]) + math::Square(q[1]) - math::Square(q[2]) - math::Square(q[3]));
  ret(1) = std::asin(as);
  ret(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                      math::Square(q[0]) - math::Square(q[1]) - math::Square(q[2]) + math::Square(q[3]));
  return true;
}

bool RPYToQuat(Eigen::Ref<Quat> ret, Eigen::Ref<Vector3 const> const &rpy) {
  RotMat R;
  RPYToRotMat(R, rpy);
  RotMatToQuat(ret, R);
  return true;
}

bool RotMatToRPY(Eigen::Ref<Vector3> ret, Eigen::Ref<RotMat const> const &R) {
  Quat q;
  RotMatToQuat(q, R);
  QuatToRPY(ret, q);
  return true;
}
/*!
 * Quaternion derivative calculation, like rqd(q, omega) in MATLAB.
 * 四元数导数的计算 the omega is expressed in body frame 角速度在身体框架中表示
 * @param ret
 * @param q
 * @param omega
 * @return
 */
bool QuatDerivative(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &q, Eigen::Ref<Vector3 const> const &omega) {
  auto quaternionDerivativeStabilization = 0.1;
  // first case in rqd
  Matrix4 Q;
  Q << q[0], -q[1], -q[2], -q[3], q[1], q[0], -q[3], q[2], q[2], q[3], q[0], -q[1], q[3], -q[2], q[1], q[0];

  Quat qq(quaternionDerivativeStabilization * omega.norm() * (1 - q.norm()), omega[0], omega[1], omega[2]);
  ret = 0.5 * Q * qq;
  return true;
}

/*!
 * Take the product of two quaternions 取两个四元数的乘积
 */
bool QuatProduct(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &q1, Eigen::Ref<Quat const> const &q2) {
  fpt_t r1 = q1[0];
  fpt_t r2 = q2[0];
  Vector3 v1(q1[1], q1[2], q1[3]);
  Vector3 v2(q2[1], q2[2], q2[3]);

  fpt_t r = r1 * r2 - v1.dot(v2);
  Vector3 v = r1 * v2 + r2 * v1 + v1.cross(v2);
  ret << r, v[0], v[1], v[2];
  return true;
}

/*!
 * Compute new quaternion given://根据角速度计算新的四元数
 * @param ret
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
bool QuatIntegrate(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &quat, Eigen::Ref<Vector3 const> const &omega,
                   fpt_t const dt) {
  Vector3 axis;
  fpt_t ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Vector3(1, 0, 0);
  }

  ang *= dt;
  Vector3 ee = std::sin(ang / 2) * axis;
  Quat quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  QuatProduct(ret, quatD, quat);
  ret = ret / ret.norm();
  return true;
}

/*!
 * Compute new quaternion given:
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
bool QuatIntegrateImplicit(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &quat,
                           Eigen::Ref<Vector3 const> const &omega, fpt_t const dt) {
  Vector3 axis;
  fpt_t ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Vector3(1, 0, 0);
  }

  ang *= dt;
  Vector3 ee = std::sin(ang / 2) * axis;
  Quat quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  QuatProduct(ret, quat, quatD);
  ret = ret / ret.norm();
  return true;
}

/*!
 * Convert a quaternion to so3.
 */
bool QuatToSO3(Eigen::Ref<Vector3> ret, Eigen::Ref<Quat const> const &q) {
  fpt_t theta = 2. * std::acos(q[0]);
  if (fabs(theta) < 1.e-6) {
    ret.setZero();
    return true;
  }

  ret[0] = theta * q[1] / std::sin(theta / 2.);
  ret[1] = theta * q[2] / std::sin(theta / 2.);
  ret[2] = theta * q[3] / std::sin(theta / 2.);
  return true;
}

bool SO3ToQuat(Eigen::Ref<Quat> ret, Eigen::Ref<Vector3 const> const &so3) {
  fpt_t theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);

  if (fabs(theta) < 1.e-6) {
    ret.setZero();
    ret[0] = 1.;
    return true;
  }
  ret[0] = cos(theta / 2.);
  ret[1] = so3[0] / theta * std::sin(theta / 2.);
  ret[2] = so3[1] / theta * std::sin(theta / 2.);
  ret[3] = so3[2] / theta * std::sin(theta / 2.);
  return true;
}
}  // namespace sdquadx::dynamics
