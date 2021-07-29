#include "dynamics/inertia.h"
#include "math/algebra.h"


namespace sdrobot::dynamics
{
  bool BuildSpatialInertia(Eigen::Ref<SpatialInertia> ret, double mass, Eigen::Ref<Vector3 const> const &com, Eigen::Ref<InertiaMat const> const &inertia)
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
}
