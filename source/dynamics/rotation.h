#pragma once

#include "dynamics/types.h"

namespace sdrobot::dynamics
{

  /*!
 * Compute rotation matrix for coordinate transformation. Note that
 * CoordinateRot(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
 * this transforms into a frame rotated by .1 radians!.
 * 计算坐标变换的旋转矩阵。请注意,
 * 坐标(坐标轴:X， .1) * v将v旋转-。1弧度-
 * 转换成一个旋转了1弧度的帧!
 */
  void CoordinateRot(RotMat &ret, CoordinateAxis axis, double theta);

  /*!
 * Go from rpy to rotation matrix.从欧拉角转矩阵
 */
  void RPYToRotMat(RotMat &ret, const Vector3 &v);

  /*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix 向量转反对称阵
 */
  void VecToSkewMat(Matrix3 &ret, const Vector3 &v);

  /*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector 反对称阵转向量
 */
  void MatToSkewVec(Vector3 &ret, const Matrix3 &m);

  /*!
 * Convert a coordinate transformation matrix to an orientation quaternion. 将坐标变换矩阵转换为方向四元数
 */
  void RotMatToQuat(Quat &ret, const RotMat &r1);

  /*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 *将四元数转换为旋转矩阵。这个矩阵表示
 *坐标转换为指定方向的具有指定方向的坐标系 通过四元数
 */
  void QuatToRotMat(RotMat &ret, const Quat &q);

  /*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
  void QuatToRPY(Vector3 &ret, const Quat &q);

  void RPYToQuat(Quat &ret, const Vector3 &rpy);

  void RotMatToRPY(Vector3 &ret, const RotMat &R);
  /*!
 * Quaternion derivative calculation, like rqd(q, omega) in MATLAB. 四元数导数的计算
 * the omega is expressed in body frame 角速度在身体框架中表示
 * @param ret
 * @param q
 * @param omega
 * @return
 */
  void QuatDerivative(Quat &ret, const Quat &q, const Vector3 &omega);

  /*!
 * Take the product of two quaternions 取两个四元数的乘积
 */
  void QuatProduct(Quat &ret, const Quat &q1, const Quat &q2);

  /*!
 * Compute new quaternion given://根据角速度计算新的四元数
 * @param ret
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  void QuatIntegrate(Quat &ret, const Quat &quat, const Vector3 &omega, double dt);

  /*!
 * Compute new quaternion given:
 * @param ret
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  void QuatIntegrateImplicit(Quat &ret, const Quat &quat, const Vector3 &omega, double dt);

  /*!
 * Convert a quaternion to so3.
 */
  void QuatToSO3(Vector3 &ret, const Quat &q);

  void SO3ToQuat(Quat &ret, Vector3 &so3);

} // namespace sdrobot::kinematics
