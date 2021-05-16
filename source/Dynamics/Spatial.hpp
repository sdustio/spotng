#pragma once

#include "Kinematics/Rotation.hpp"

namespace sd::dynamics
{
  enum class JointType
  {
    Prismatic,
    Revolute,
    FloatingBase,
    Nothing
  };

  /*!
 * Representation of Rigid Body Inertia as a 6x6 Spatial Inertia Tensor 用6x6空间惯性张量表示刚体惯性
 */
  template <typename T>
  class SpatialInertia
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
   * Construct spatial inertia from mass, center of mass, and 3x3 rotational 从质量、质心和3 * 3的转动惯性来构造空间惯量
   * inertia
   */
    explicit SpatialInertia(T mass, const Vec3<T> &com, const Mat3<T> &inertia)
    {
      Mat3<T> cSkew = sd::kinematics::VecToSkewMat(com); //质心向量转反对称矩阵

      mSpatialInertia.template topLeftCorner<3, 3>() =
          inertia + mass * cSkew * cSkew.transpose();
      mSpatialInertia.template topRightCorner<3, 3>() = mass * cSkew;
      mSpatialInertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
      mSpatialInertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
    }

    /*!
   * Construct spatial inertia from 6x6 matrix 从6x6矩阵构造空间惯量
   */
    explicit SpatialInertia(const Mat6<T> &inertia)
    {
      mSpatialInertia = inertia;
    }

    /*!
   * If no argument is given, zero. 如果没有给出参数，则为零。
   */
    SpatialInertia()
    {
      mSpatialInertia = Mat6<T>::Zero();
    }

    /*!
   * Construct spatial inertia from mass property vector 从质量特性向量出发构造空间惯量
   */
    explicit SpatialInertia(const MassProperties<T> &a)
    {
      mSpatialInertia(0, 0) = a(4);
      mSpatialInertia(0, 1) = a(9);
      mSpatialInertia(0, 2) = a(8);
      mSpatialInertia(1, 0) = a(9);
      mSpatialInertia(1, 1) = a(5);
      mSpatialInertia(1, 2) = a(7);
      mSpatialInertia(2, 0) = a(8);
      mSpatialInertia(2, 1) = a(7);
      mSpatialInertia(2, 2) = a(6);
      Mat3<T> cSkew = sd::kinematics::VecToSkewMat(Vec3<T>(a(1), a(2), a(3)));
      mSpatialInertia.template topRightCorner<3, 3>() = cSkew;
      mSpatialInertia.template bottomLeftCorner<3, 3>() = cSkew.transpose();
      mSpatialInertia.template bottomRightCorner<3, 3>() = a(0) * Mat3<T>::Identity();
    }

    /*!
   * Construct spatial inertia from pseudo-inertia. This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   *由伪惯性构造空间惯性。这是描述在线性矩阵不等式的物理一致的惯性参数识别:一个统计角度的质量分布，由温辛，金，斯洛廷
   */
    explicit SpatialInertia(const Mat4<T> &P)
    {
      T m = P(3, 3);
      Vec3<T> h = P.template topRightCorner<3, 1>();
      Mat3<T> E = P.template topLeftCorner<3, 3>();
      Mat3<T> Ibar = E.trace() * Mat3<T>::Identity() - E;
      mSpatialInertia.template topLeftCorner<3, 3>() = Ibar;
      mSpatialInertia.template topRightCorner<3, 3>() = sd::kinematics::VecToSkewMat(h);
      mSpatialInertia.template bottomLeftCorner<3, 3>() = sd::kinematics::VecToSkewMat(h).transpose();
      mSpatialInertia.template bottomRightCorner<3, 3>() = m * Mat3<T>::Identity();
    }

    /*!
   * Get 6x6 spatial inertia
   得到6x6的空间惯量
   */
    const Mat6<T>& GetSpatialInertia() const { return mSpatialInertia; }


    bool SetSpatialInertia(const Mat6<T> &mat) {
      mSpatialInertia = mat;
      return true;
      }

    bool AddSpatialInertia(const Mat6<T> &mat) {
      mSpatialInertia += mat;
      return true;
      }

    /*!
   * Convert spatial inertia to mass property vector
   将空间惯性转化为质量特性矢量
   */
    const MassProperties<T> CalcMassProperties() const {
      MassProperties<T> m;
      Vec3<T> h = sd::kinematics::MatToSkewVec(mSpatialInertia.template topRightCorner<3, 3>());
      m << mSpatialInertia(5, 5), h(0), h(1), h(2), mSpatialInertia(0, 0), mSpatialInertia(1, 1),
          mSpatialInertia(2, 2), mSpatialInertia(2, 1), mSpatialInertia(2, 0), mSpatialInertia(1, 0);
      return m;
    }

    /*!
   * Get mass 得到质量
   */
    const T CalcMass() const { return mSpatialInertia(5, 5); }

    /*!
   * Get center of mass location 得到质心的位置
   */
    const Vec3<T> CalcCOM() const{
      T m = CalcMass();
      Mat3<T> mcSkew = mSpatialInertia.template topRightCorner<3, 3>();
      return sd::kinematics::MatToSkewVec(mcSkew) / m;
    }

    /*!
   * Get 3x3 rotational inertia 得到3 * 3的转动惯量
   */
    const Mat3<T> CalcInertiaMat() const {
      T m = CalcMass();
      Mat3<T> mcSkew = mSpatialInertia.template topRightCorner<3, 3>();
      return mSpatialInertia.template topLeftCorner<3, 3>() -
                      mcSkew * mcSkew.transpose() / m;
    }

    /*!
   * Convert to 4x4 pseudo-inertia matrix.  This is described in
   * Linear Matrix Inequalities for Physically Consistent Inertial Parameter
   *   Identification: A Statistical Perspective on the Mass Distribution, by
   *   Wensing, Kim, Slotine
   */
    Mat4<T> CalcPseudoInertiaMat()
    {
      Vec3<T> h = sd::kinematics::MatToSkewVec(mSpatialInertia.template topRightCorner<3, 3>());
      Mat3<T> Ibar = mSpatialInertia.template topLeftCorner<3, 3>();
      T m = mSpatialInertia(5, 5);
      Mat4<T> P;
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
    SpatialInertia FlipAlongAxis(sd::kinematics::CoordinateAxis axis)
    {
      Mat4<T> P = CalcPseudoInertiaMat();
      Mat4<T> X = Mat4<T>::Identity();
      if (axis == sd::kinematics::CoordinateAxis::X)
        X(0, 0) = -1;
      else if (axis == sd::kinematics::CoordinateAxis::Y)
        X(1, 1) = -1;
      else if (axis == sd::kinematics::CoordinateAxis::Z)
        X(2, 2) = -1;
      P = X * P * X;
      return SpatialInertia(P);
    }

  private:
    Mat6<T> mSpatialInertia;

  };

  template class SpatialInertia<double>; //for compile check

} // namespace sd::dynamics
