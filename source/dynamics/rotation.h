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
  void CoordinateRot(Eigen::Ref<RotMat> ret, CoordinateAxis const axis, fptype const theta);

  /*!
 * Go from rpy to rotation matrix.从欧拉角转矩阵
 */
  void RPYToRotMat(Eigen::Ref<RotMat> ret, Eigen::Ref<Vector3 const> const &v);

  /*!
 * Convert a coordinate transformation matrix to an orientation quaternion. 将坐标变换矩阵转换为方向四元数
 */
  void RotMatToQuat(Eigen::Ref<Quat> ret, Eigen::Ref<RotMat const> const &r1);

  /*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 *将四元数转换为旋转矩阵。这个矩阵表示
 *坐标转换为指定方向的具有指定方向的坐标系 通过四元数
 */
  void QuatToRotMat(Eigen::Ref<RotMat> ret, Eigen::Ref<Quat const> const &q);

  /*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
  void QuatToRPY(Eigen::Ref<Vector3> ret, Eigen::Ref<Quat const> const &q);

  void RPYToQuat(Eigen::Ref<Quat> ret, Eigen::Ref<Vector3 const> const &rpy);

  void RotMatToRPY(Eigen::Ref<Vector3> ret, Eigen::Ref<RotMat const> const &R);
  /*!
 * Quaternion derivative calculation, like rqd(q, omega) in MATLAB. 四元数导数的计算
 * the omega is expressed in body frame 角速度在身体框架中表示
 * @param ret
 * @param q
 * @param omega
 * @return
 */
  void QuatDerivative(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &q, Eigen::Ref<Vector3 const> const &omega);

  /*!
 * Take the product of two quaternions 取两个四元数的乘积
 */
  void QuatProduct(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &q1, Eigen::Ref<Quat const> const &q2);

  /*!
 * Compute new quaternion given://根据角速度计算新的四元数
 * @param ret
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  void QuatIntegrate(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &quat, Eigen::Ref<Vector3 const> const &omega, fptype const dt);

  /*!
 * Compute new quaternion given:
 * @param ret
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  void QuatIntegrateImplicit(Eigen::Ref<Quat> ret, Eigen::Ref<Quat const> const &quat, Eigen::Ref<Vector3 const> const &omega, fptype const dt);

  /*!
 * Convert a quaternion to so3.
 */
  void QuatToSO3(Eigen::Ref<Vector3> ret, Eigen::Ref<Quat const> const &q);

  void SO3ToQuat(Eigen::Ref<Quat> ret, Eigen::Ref<Vector3 const> const &so3);

} // namespace sdrobot::kinematics
