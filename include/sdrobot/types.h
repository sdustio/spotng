#pragma once

#include <eigen3/Eigen/Dense>

namespace sdrobot
{
    constexpr double kZeroEpsilon = 1.e-10;

    using Vector2 = Eigen::Vector2d;
    using Vector3 = Eigen::Vector3d;
    using Vector4 = Eigen::Vector4d;
    using VectorX = Eigen::VectorXd;

    using RowVector2 = Eigen::RowVector2d;
    using RowVector3 = Eigen::RowVector3d;
    using RowVector4 = Eigen::RowVector4d;
    using RowVectorX = Eigen::RowVectorXd;

    using Matrix2 = Eigen::Matrix2d;
    using Matrix3 = Eigen::Matrix3d;
    using Matrix4 = Eigen::Matrix4d;

    using Matrix2X = Eigen::Matrix2Xd;
    using Matrix3X = Eigen::Matrix3Xd;
    using Matrix4X = Eigen::Matrix4Xd;

    using MatrixX2 = Eigen::MatrixX2d;
    using MatrixX3 = Eigen::MatrixX3d;
    using MatrixX4 = Eigen::MatrixX4d;

    using MatrixX = Eigen::MatrixXd;

    // 6x1 Vector
    using Vector6 = Eigen::Matrix<double, 6, 1>;

    // 10x1 Vector
    using Vector10 = Eigen::Matrix<double, 10, 1>;

    // 12x1 Vector
    using Vector12 = Eigen::Matrix<double, 12, 1>;

    // 18x1 Vector
    using Vector18 = Eigen::Matrix<double, 18, 1>;

    // 28x1 vector
    using Vector28 = Eigen::Matrix<double, 28, 1>;

    // 6x6 Matrix
    using Matrix6 = Eigen::Matrix<double, 6, 6>;

    // 12x12 Matrix
    using Matrix12 = Eigen::Matrix<double, 12, 12>;

    // 18x18 Matrix
    using Matrix18 = Eigen::Matrix<double, 18, 18>;

    // 28x28 Matrix
    using Matrix28 = Eigen::Matrix<double, 28, 28>;

    // 3x4 Matrix
    using Matrix3x4 = Eigen::Matrix<double, 3, 4>;

    // 2x3 Matrix
    using Matrix2x3 = Eigen::Matrix<double, 2, 3>;

}
