#pragma once

#include "externlib/eigen.h"

namespace sdquadx::dynamics {

enum class JointType { Prismatic, Revolute, FloatingBase, Nothing };

enum class CoordinateAxis { X, Y, Z };

// Rotation Matrix
using RotMat = Matrix3;

// Quaternions
using Quat = Vector4;

// Spatial Mass Matrix
using SpatialInertia = Matrix6;

// Spatial Vector (6x1, all subspaces)
using SpatialVec = Vector6;

// Spatial Transform (6x6)
using SpatialXform = Matrix6;

// 10x1 Vector
using MassProperties = Eigen::Matrix<fpt_t, 10, 1>;

using InertiaTensor = Matrix3;  // InertiaMat

using PseudoInertiaTensor = Matrix4;  // PseudoInertiaMat

}  // namespace sdquadx::dynamics
