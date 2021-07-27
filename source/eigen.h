#pragma once

#include <Eigen/Dense>

#include "sdrobot/types.h"


namespace sdrobot
{
    using Vector3 = Eigen::Matrix<fptype, 3, 1>;
    using Vector4 = Eigen::Matrix<fptype, 4, 1>;
    using VectorX = Eigen::Matrix<fptype, Eigen::Dynamic, 1>;

    using Matrix3 = Eigen::Matrix<fptype, 3, 3>;
    using Matrix4 = Eigen::Matrix<fptype, 4, 4>;
    using MatrixX = Eigen::Matrix<fptype, Eigen::Dynamic, Eigen::Dynamic>;

    // 6x1 Vector
    using Vector6 = Eigen::Matrix<fptype, 6, 1>;

    // 10x1 Vector
    using Vector10 = Eigen::Matrix<fptype, 10, 1>;

    // 12x1 Vector
    using Vector12 = Eigen::Matrix<fptype, 12, 1>;

    // 18x1 Vector
    using Vector18 = Eigen::Matrix<fptype, 18, 1>;

    // 28x1 vector
    using Vector28 = Eigen::Matrix<fptype, 28, 1>;

    // 6x6 Matrix
    using Matrix6 = Eigen::Matrix<fptype, 6, 6>;

    // 12x12 Matrix
    using Matrix12 = Eigen::Matrix<fptype, 12, 12>;

    // 18x18 Matrix
    using Matrix18 = Eigen::Matrix<fptype, 18, 18>;

    // 28x28 Matrix
    using Matrix28 = Eigen::Matrix<fptype, 28, 28>;

    // 3x4 Matrix
    using Matrix3x4 = Eigen::Matrix<fptype, 3, 4>;

    // 2x3 Matrix
    using Matrix2x3 = Eigen::Matrix<fptype, 2, 3>;

}
