#pragma once

#include <eigen3/Eigen/Dense>

namespace sdrobot
{
    constexpr double kZeroEpsilon = 1.e-10;

    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Vector4d;
    using Eigen::VectorXd;

    using Eigen::RowVector2d;
    using Eigen::RowVector3d;
    using Eigen::RowVector4d;
    using Eigen::RowVectorXd;

    using Eigen::Matrix2d;
    using Eigen::Matrix3d;
    using Eigen::Matrix4d;

    using Eigen::Matrix2Xd;
    using Eigen::Matrix3Xd;
    using Eigen::Matrix4Xd;

    using Eigen::MatrixX2d;
    using Eigen::MatrixX3d;
    using Eigen::MatrixX4d;

    using Eigen::MatrixXd;

    // 6x1 Vector
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    // 10x1 Vector
    using Vector10d = Eigen::Matrix<double, 10, 1>;

    // 12x1 Vector
    using Vector12d = Eigen::Matrix<double, 12, 1>;

    // 18x1 Vector
    using Vector18d = Eigen::Matrix<double, 18, 1>;

    // 28x1 vector
    using Vector28d = Eigen::Matrix<double, 28, 1>;

    // 6x6 Matrix
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    // 12x12 Matrix
    using Matrix12d = Eigen::Matrix<double, 12, 12>;

    // 18x18 Matrix
    using Matrix18d = Eigen::Matrix<double, 18, 18>;

    // 28x28 Matrix
    using Matrix28d = Eigen::Matrix<double, 28, 28>;

    // 3x4 Matrix
    using Matrix3x4d = Eigen::Matrix<double, 3, 4>;

    // 2x3 Matrix
    using Matrix2x3d = Eigen::Matrix<double, 2, 3>;

}
