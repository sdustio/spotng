#pragma once

#include <Eigen/Dense>

#include "sdrobot/types.h"

namespace sdrobot {
using Vector3 = Eigen::Matrix<fpt_t, 3, 1>;
using Vector4 = Eigen::Matrix<fpt_t, 4, 1>;
using Vector6 = Eigen::Matrix<fpt_t, 6, 1>;
using VectorX = Eigen::Matrix<fpt_t, Eigen::Dynamic, 1>;

using Matrix3 = Eigen::Matrix<fpt_t, 3, 3>;
using Matrix4 = Eigen::Matrix<fpt_t, 4, 4>;
using Matrix6 = Eigen::Matrix<fpt_t, 6, 6>;
using MatrixX = Eigen::Matrix<fpt_t, Eigen::Dynamic, Eigen::Dynamic>;

// 3x4 Matrix
using Matrix3x4 = Eigen::Matrix<fpt_t, 3, 4>;

// 2x3 Matrix
using Matrix2x3 = Eigen::Matrix<fpt_t, 2, 3>;

Eigen::Ref<Vector3> ToEigenTp(SdVector3f &v);
Eigen::Ref<Matrix3> ToEigenTp(SdMatrix3f &m);
Eigen::Ref<Vector4> ToEigenTp(SdVector4f &v);
Eigen::Ref<Matrix4> ToEigenTp(SdMatrix4f &m);
Eigen::Ref<Vector6> ToEigenTp(SdVector6f &v);
Eigen::Ref<Matrix6> ToEigenTp(SdMatrix6f &m);

Eigen::Ref<VectorX> ToEigenTp(SdVectorXf &v, int const row);
Eigen::Ref<MatrixX> ToEigenTp(SdMatrixXf &m, int const row, int const col);

Eigen::Ref<Vector3 const> ToConstEigenTp(SdVector3f const &v);
Eigen::Ref<Matrix3 const> ToConstEigenTp(SdMatrix3f const &m);
Eigen::Ref<Vector4 const> ToConstEigenTp(SdVector4f const &v);
Eigen::Ref<Matrix4 const> ToConstEigenTp(SdMatrix4f const &m);
Eigen::Ref<Vector6 const> ToConstEigenTp(SdVector6f const &v);
Eigen::Ref<Matrix6 const> ToConstEigenTp(SdMatrix6f const &m);

Eigen::Ref<VectorX const> ToConstEigenTp(SdVectorXf const &v, int const row);
Eigen::Ref<MatrixX const> ToConstEigenTp(SdMatrixXf const &m, int const row,
                                         int const col);
}  // namespace sdrobot
