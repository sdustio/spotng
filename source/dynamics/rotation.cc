#include <cmath>

#include "sdrobot/dynamics/math.h"
#include "sdrobot/dynamics/rotation.h"

namespace sdrobot::dynamics
{
  /*!
 * Convert radians to degrees 转化弧度到度数
 */
  double RadToDeg(double rad)
  {
    return rad * 180.0 / M_PI;
  }

  /*!
 * Convert degrees to radians
 */
  double DegToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  /*!
 * Compute rotation matrix for coordinate transformation. Note that
 * CoordinateRot(CoordinateAxis:X, .1) * v will rotate v by -.1 radians -
 * this transforms into a frame rotated by .1 radians!.
 * 计算坐标变换的旋转矩阵。请注意,
 * 坐标(坐标轴:X， .1) * v将v旋转-。1弧度-
 * 转换成一个旋转了1弧度的帧!
 */
  RotMat CoordinateRot(CoordinateAxis axis, double theta)
  {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;

    if (axis == CoordinateAxis::X)
    {
      R << 1, 0, 0, 0, c, s, 0, -s, c;
    }
    else if (axis == CoordinateAxis::Y)
    {
      R << c, 0, -s, 0, 1, 0, s, 0, c;
    }
    else if (axis == CoordinateAxis::Z)
    {
      R << c, s, 0, -s, c, 0, 0, 0, 1;
    }

    return R;
  }

  /*!
 * Go from rpy to rotation matrix.从欧拉角转矩阵
 */
  RotMat RPYToRotMat(const Vector3d &v)
  {
    RotMat m = CoordinateRot(CoordinateAxis::X, v[0]) *
               CoordinateRot(CoordinateAxis::Y, v[1]) *
               CoordinateRot(CoordinateAxis::Z, v[2]);
    return m;
  }

  /*!
 * Convert a 3x1 vector to a skew-symmetric 3x3 matrix 向量转反对称阵
 */
  Matrix3d VecToSkewMat(const Vector3d &v)
  {
    Matrix3d m;
    m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return m;
  }

  /*!
 * Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector 反对称阵转向量
 */
  Vector3d MatToSkewVec(const Matrix3d &m)
  {
    return 0.5 * Vector3d(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0),
                          (m(1, 0) - m(0, 1)));
  }

  /*!
 * Convert a coordinate transformation matrix to an orientation quaternion. 将坐标变换矩阵转换为方向四元数
 */
  Quat RotMatToQuat(const RotMat &r1)
  {
    Quat q;
    RotMat r = r1.transpose();
    double tr = r.trace();
    if (tr > 0.0)
    {
      double S = sqrt(tr + 1.0) * 2.0;
      q(0) = 0.25 * S;
      q(1) = (r(2, 1) - r(1, 2)) / S;
      q(2) = (r(0, 2) - r(2, 0)) / S;
      q(3) = (r(1, 0) - r(0, 1)) / S;
    }
    else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2)))
    {
      double S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
      q(0) = (r(2, 1) - r(1, 2)) / S;
      q(1) = 0.25 * S;
      q(2) = (r(0, 1) + r(1, 0)) / S;
      q(3) = (r(0, 2) + r(2, 0)) / S;
    }
    else if (r(1, 1) > r(2, 2))
    {
      double S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
      q(0) = (r(0, 2) - r(2, 0)) / S;
      q(1) = (r(0, 1) + r(1, 0)) / S;
      q(2) = 0.25 * S;
      q(3) = (r(1, 2) + r(2, 1)) / S;
    }
    else
    {
      double S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
      q(0) = (r(1, 0) - r(0, 1)) / S;
      q(1) = (r(0, 2) + r(2, 0)) / S;
      q(2) = (r(1, 2) + r(2, 1)) / S;
      q(3) = 0.25 * S;
    }
    return q;
  }

  /*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 *将四元数转换为旋转矩阵。这个矩阵表示
 *坐标转换为指定方向的具有指定方向的坐标系 通过四元数
 */
  RotMat QuatToRotMat(const Quat &q)
  {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat R;

    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
        2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
        1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
        2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
        1 - 2 * (e1 * e1 + e2 * e2);
    R.transposeInPlace();
    return R;
  }

  /*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
  Vector3d QuatToRPY(const Quat &q)
  {
    Vector3d rpy;
    double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) =
        std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                   Square(q[0]) + Square(q[1]) - Square(q[2]) - Square(q[3]));
    rpy(1) = std::asin(as);
    rpy(0) =
        std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                   Square(q[0]) - Square(q[1]) - Square(q[2]) + Square(q[3]));
    return rpy;
  }

  Quat RPYToQuat(const Vector3d &rpy)
  {
    RotMat R = RPYToRotMat(rpy);
    Quat q = RotMatToQuat(R);
    return q;
  }

  Vector3d RotMatToRPY(const RotMat &R)
  {
    Quat q = RotMatToQuat(R);
    Vector3d rpy = QuatToRPY(q);
    return rpy;
  }
  /*!
 * Quaternion derivative calculation, like rqd(q, omega) in MATLAB. 四元数导数的计算
 * the omega is expressed in body frame 角速度在身体框架中表示
 * @param q
 * @param omega
 * @return
 */
  Quat QuatDerivative(const Quat &q, const Vector3d &omega)
  {
    auto quaternionDerivativeStabilization = 0.1;
    // first case in rqd
    Matrix4d Q;
    Q << q[0], -q[1], -q[2], -q[3], q[1], q[0], -q[3], q[2], q[2], q[3], q[0],
        -q[1], q[3], -q[2], q[1], q[0];

    Quat qq(
        quaternionDerivativeStabilization * omega.norm() * (1 - q.norm()),
        omega[0], omega[1], omega[2]);
    Quat dq = 0.5 * Q * qq;
    return dq;
  }

  /*!
 * Take the product of two quaternions 取两个四元数的乘积
 */
  Quat QuatProduct(const Quat &q1, const Quat &q2)
  {
    double r1 = q1[0];
    double r2 = q2[0];
    Vector3d v1(q1[1], q1[2], q1[3]);
    Vector3d v2(q2[1], q2[2], q2[3]);

    double r = r1 * r2 - v1.dot(v2);
    Vector3d v = r1 * v2 + r2 * v1 + v1.cross(v2);
    Quat q(r, v[0], v[1], v[2]);
    return q;
  }

  /*!
 * Compute new quaternion given://根据角速度计算新的四元数
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  Quat QuatIntegrate(const Quat &quat, const Vector3d &omega, double dt)
  {
    Vector3d axis;
    double ang = omega.norm();
    if (ang > 0)
    {
      axis = omega / ang;
    }
    else
    {
      axis = Vector3d(1, 0, 0);
    }

    ang *= dt;
    Vector3d ee = std::sin(ang / 2) * axis;
    Quat quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

    Quat quatNew = QuatProduct(quatD, quat);
    quatNew = quatNew / quatNew.norm();
    return quatNew;
  }

  /*!
 * Compute new quaternion given:
 * @param quat The old quaternion
 * @param omega The angular velocity (IN INERTIAL COORDINATES!)
 * @param dt The timestep
 * @return
 */
  Quat QuatIntegrateImplicit(const Quat &quat, const Vector3d &omega, double dt)
  {
    Vector3d axis;
    double ang = omega.norm();
    if (ang > 0)
    {
      axis = omega / ang;
    }
    else
    {
      axis = Vector3d(1, 0, 0);
    }

    ang *= dt;
    Vector3d ee = std::sin(ang / 2) * axis;
    Quat quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

    Quat quatNew = QuatProduct(quat, quatD);
    quatNew = quatNew / quatNew.norm();
    return quatNew;
  }

  /*!
 * Convert a quaternion to so3.
 */
  Vector3d QuatToSO3(const Quat &q)
  {
    Vector3d so3;
    double theta = 2. * std::acos(q[0]);
    so3[0] = theta * q[1] / std::sin(theta / 2.);
    so3[1] = theta * q[2] / std::sin(theta / 2.);
    so3[2] = theta * q[3] / std::sin(theta / 2.);
    return so3;
  }

  Quat SO3ToQuat(Vector3d &so3)
  {
    Quat quat;

    double theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);

    if (fabs(theta) < 1.e-6)
    {
      quat.setZero();
      quat[0] = 1.;
      return quat;
    }
    quat[0] = cos(theta / 2.);
    quat[1] = so3[0] / theta * sin(theta / 2.);
    quat[2] = so3[1] / theta * sin(theta / 2.);
    quat[3] = so3[2] / theta * sin(theta / 2.);
    return quat;
  }
} // namespace sdrobot::kinematics
