#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

namespace sd
{

    // 2x1 Vector
    template <typename T>
    using Vec2 = Eigen::Matrix<T, 2, 1>;

    // 3x1 Vector
    template <typename T>
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    // 4x1 Vector
    template <typename T>
    using Vec4 = Eigen::Matrix<T, 4, 1>;

    // 6x1 Vector
    template <typename T>
    using Vec6 = Eigen::Matrix<T, 6, 1>;

    // 10x1 Vector
    template <typename T>
    using Vec10 = Eigen::Matrix<T, 10, 1>;

    // 12x1 Vector
    template <typename T>
    using Vec12 = Eigen::Matrix<T, 12, 1>;

    // 18x1 Vector
    template <typename T>
    using Vec18 = Eigen::Matrix<T, 18, 1>;

    // 28x1 vector
    template <typename T>
    using Vec28 = Eigen::Matrix<T, 28, 1>;

    // 3x3 Matrix
    template <typename T>
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    // 6x6 Matrix
    template <typename T>
    using Mat6 = Eigen::Matrix<T, 6, 6>;

    // 12x12 Matrix
    template <typename T>
    using Mat12 = Eigen::Matrix<T, 12, 12>;

    // 18x18 Matrix
    template <typename T>
    using Mat18 = Eigen::Matrix<T, 18, 18>;

    // 28x28 Matrix
    template <typename T>
    using Mat28 = Eigen::Matrix<T, 28, 28>;

    // 3x4 Matrix
    template <typename T>
    using Mat34 = Eigen::Matrix<T, 3, 4>;

    // 3x4 Matrix
    template <typename T>
    using Mat23 = Eigen::Matrix<T, 2, 3>;

    // 4x4 Matrix
    template <typename T>
    using Mat4 = Eigen::Matrix<T, 4, 4>;

    // Dynamically sized vector
    template <typename T>
    using DVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    // Dynamically sized matrix
    template <typename T>
    using DMat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    // Dynamically sized matrix with spatial vector columns
    template <typename T>
    using D6Mat = Eigen::Matrix<T, 6, Eigen::Dynamic>;

    // Dynamically sized matrix with cartesian vector columns
    template <typename T>
    using D3Mat = Eigen::Matrix<T, 3, Eigen::Dynamic>;

    // std::vector (a list) of Eigen things
    template <typename T>
    using VectorAligned = std::vector<T, Eigen::aligned_allocator<T>>;

}
