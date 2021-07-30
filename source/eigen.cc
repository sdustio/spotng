#include "eigen.h"

namespace sdrobot
{
  Eigen::Ref<Vector3> ToEigenMatrix(SdVector3f &v)
  {
    Eigen::Map<Vector3> egv(v.data());
    return egv;
  }
  Eigen::Ref<Matrix3> ToEigenMatrix(SdMatrix3f &m)
  {
    Eigen::Map<Matrix3> egm(m.data());
    return egm;
  }
  Eigen::Ref<Vector4> ToEigenMatrix(SdVector4f &v)
  {
    Eigen::Map<Vector4> egv(v.data());
    return egv;
  }
  Eigen::Ref<Matrix4> ToEigenMatrix(SdMatrix4f &m)
  {
    Eigen::Map<Matrix4> egm(m.data());
    return egm;
  }
  Eigen::Ref<Vector6> ToEigenMatrix(SdVector6f &v)
  {
    Eigen::Map<Vector6> egv(v.data());
    return egv;
  }
  Eigen::Ref<Matrix6> ToEigenMatrix(SdMatrix6f &m)
  {
    Eigen::Map<Matrix6> egm(m.data());
    return egm;
  }

  Eigen::Ref<VectorX> ToEigenMatrix(SdVectorXf &v, int const row)
  {
    if (v.size() < unsigned(row))
    {
      throw("size of v is smaller than row");
    }

    Eigen::Map<VectorX> egv(v.data(), row);
    return egv;
  }
  Eigen::Ref<MatrixX> ToEigenMatrix(SdMatrixXf &m, int const row, int const col)
  {
    if (m.size() < unsigned(row * col))
    {
      throw("size of m is smaller than row * col");
    }
    Eigen::Map<MatrixX> egm(m.data(), row, col);
    return egm;
  }


  Eigen::Ref<Vector3 const> ToConstEigenMatrix(SdVector3f const &v)
  {
    Eigen::Map<Vector3 const> egv(v.data());
    return egv;
  }
  Eigen::Ref<Matrix3 const> ToConstEigenMatrix(SdMatrix3f const &m)
  {
    Eigen::Map<Matrix3 const> egm(m.data());
    return egm;
  }
  Eigen::Ref<Vector4 const> ToConstEigenMatrix(SdVector4f const &v)
  {
    Eigen::Map<Vector4 const> egv(v.data());
    return egv;
  }
  Eigen::Ref<Matrix4 const> ToConstEigenMatrix(SdMatrix4f const &m)
  {
    Eigen::Map<Matrix4 const> egm(m.data());
    return egm;
  }
  Eigen::Ref<Vector6 const> ToConstEigenMatrix(SdVector6f const &v)
  {
    Eigen::Map<Vector6 const> egv(v.data());
    return egv;
  }
  Eigen::Ref<Matrix6 const> ToConstEigenMatrix(SdMatrix6f const &m)
  {
    Eigen::Map<Matrix6 const> egm(m.data());
    return egm;
  }

  Eigen::Ref<VectorX const> ToConstEigenMatrix(SdVectorXf const &v, int const row)
  {
    if (v.size() < unsigned(row))
    {
      throw("size of v is smaller than row");
    }

    Eigen::Map<VectorX const> egv(v.data(), row);
    return egv;
  }
  Eigen::Ref<MatrixX const> ToConstEigenMatrix(SdMatrixXf const &m, int const row, int const col)
  {
    if (m.size() < unsigned(row * col))
    {
      throw("size of m is smaller than row * col");
    }
    Eigen::Map<MatrixX const> egm(m.data(), row, col);
    return egm;
  }
}
