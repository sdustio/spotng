#pragma once

#include "sd/dynamics/kinematics.h"

namespace sd::dynamics
{

  /*!
 * Square a number
 */
  double Square(double a);

  /*!
 * Convert radians to degrees 转化弧度到度数
 */
  double RadToDeg(double rad);

  /*!
 * Convert degrees to radians
 */
  double DegToRad(double deg);

  /*!
 * Compute rotation matrix for coordinate transformation. Note that
 * CoordinateRot(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
 * this transforms into a frame rotated by .1 radians!.
 * 计算坐标变换的旋转矩阵。请注意,
 * 坐标(坐标轴:X， .1) * v将v旋转-。1弧度-
 * 转换成一个旋转了1弧度的帧!
 */
  RotMat CoordinateRot(CoordinateAxis axis, double theta);

  /*!
 * Go from rpy to rotation matrix.从欧拉角转矩阵
 */
  RotMat RPYToRotMat(const Vector3d &v);

  /*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix 向量转反对称阵
 */
  Matrix3d VecToSkewMat(const Vector3d &v);

  /*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector 反对称阵转向量
 */
  Vector3d MatToSkewVec(const Matrix3d &m);

  /*!
 * Convert a coordinate transformation matrix to an orientation quaternion. 将坐标变换矩阵转换为方向四元数
 */
  Quat RotMatToQuat(const RotMat &r1);

  /*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 *将四元数转换为旋转矩阵。这个矩阵表示
 *坐标转换为指定方向的具有指定方向的坐标系 通过四元数
 */
  RotMat QuatToRotMat(const Quat &q);

  /*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
  Vector3d QuatToRPY(const Quat &q);

  Quat RPYToQuat(const Vector3d &rpy);

  Vector3d RotMatToRPY(const RotMat &R);
  /*!
 * Quaternion derivative calculation, like rqd(q, omega) in MATLAB. 四元数导数的计算
 * the omega is expressed in body frame 角速度在身体框架中表示
 * @param q
 * @param omega
 * @return
 */
  Quat QuatDerivative(const Quat &q, const Vector3d &omega);

  /*!
 * Take the product of two quaternions 取两个四元数的乘积
 */
  Quat QuatProduct(const Quat &q1, const Quat &q2);

  /*!
 * Compute new quaternion given://根据角速度计算新的四元数
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  Quat QuatIntegrate(const Quat &quat, const Vector3d &omega, double dt);

  /*!
 * Compute new quaternion given:
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  Quat QuatIntegrateImplicit(const Quat &quat, const Vector3d &omega, double dt);

  /*!
 * Convert a quaternion to so3.
 */
  Vector3d QuatToSO3(const Quat &q);

  Quat SO3ToQuat(Vector3d &so3);

} // namespace sd::kinematics
