#include "sd/dynamics/rotation.h"
#include "sd/dynamics/spatial.h"

namespace sd::dynamics
{
  /*!
  * Calculate the spatial coordinate transform from A to B where B is rotate by
  * theta about axis.
  */
  SpatialXform SpatialRotation(CoordinateAxis axis, double theta)
  {
    RotMat R = CoordinateRot(axis, theta);
    SpatialXform X = SpatialXform::Zero();
    X.topLeftCorner<3, 3>() = R;
    X.bottomRightCorner<3, 3>() = R;
    return X;
  }

  /*!
 * Compute the spatial motion cross product matrix.
 * Prefer MotionCrossProduct when possible.
 */
  Matrix6d MotionCrossMatrix(const Vector6d &v)
  {
    Matrix6d m;
    m << 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0, 0, 0,
        0,

        0, -v(5), v(4), 0, -v(2), v(1), v(5), 0, -v(3), v(2), 0, -v(0), -v(4),
        v(3), 0, -v(1), v(0), 0;
    return m;
  }

  /*!
 * Compute spatial force cross product matrix.
 * Prefer ForceCrossProduct when possible
 */
  Matrix6d ForceCrossMatrix(const Vector6d &v)
  {
    Matrix6d f;
    f << 0, -v(2), v(1), 0, -v(5), v(4), v(2), 0, -v(0), v(5), 0, -v(3), -v(1),
        v(0), 0, -v(4), v(3), 0, 0, 0, 0, 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0),
        0, 0, 0, -v(1), v(0), 0;
    return f;
  }

  /*!
 * Compute spatial motion cross product.  Faster than the matrix multiplication
 * version
 */
  SpatialVec MotionCrossProduct(const Vector6d &a, const Vector6d &b)
  {
    SpatialVec mv;
    mv << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2),
        a(0) * b(1) - a(1) * b(0),
        a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1),
        a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
        a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
    return mv;
  }

  /*!
 * Compute spatial force cross product.  Faster than the matrix multiplication
 * version
 */
  SpatialVec ForceCrossProduct(const Vector6d &a, const Vector6d &b)
  {
    SpatialVec mv;
    mv << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4),
        b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
        b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3),
        b(5) * a(1) - b(4) * a(2), b(3) * a(2) - b(5) * a(0),
        b(4) * a(0) - b(3) * a(1);
    return mv;
  }

  /*!
 * Convert a spatial transform to a homogeneous coordinate transformation
 */
  Matrix4d SpatialXformToHomogeneous(const SpatialXform &X)
  {
    Matrix4d H = Matrix4d::Zero();
    RotMat R = X.topLeftCorner<3, 3>();
    Matrix3d skewR = X.bottomLeftCorner<3, 3>();
    H.topLeftCorner<3, 3>() = R;
    H.topRightCorner<3, 1>() = MatToSkewVec(skewR * R.transpose());
    H(3, 3) = 1;
    return H;
  }

  /*!
 * Convert a homogeneous coordinate transformation to a spatial one
 */
  SpatialXform HomogeneousToSpatialXform(const Matrix4d &H)
  {
    Matrix3d R = H.topLeftCorner<3, 3>();
    Vector3d translate = H.topRightCorner<3, 1>();
    SpatialXform X = SpatialXform::Zero();
    X.topLeftCorner<3, 3>() = R;
    X.bottomLeftCorner<3, 3>() = VecToSkewMat(translate) * R;
    X.bottomRightCorner<3, 3>() = R;
    return X;
  }

  /*!
 * Create spatial coordinate transformation from rotation and translation
 */
  SpatialXform CreateSpatialXform(const RotMat &R, const Vector3d &r)
  {
    SpatialXform X = SpatialXform::Zero();
    X.topLeftCorner<3, 3>() = R;
    X.bottomRightCorner<3, 3>() = R;
    X.bottomLeftCorner<3, 3>() = -R * VecToSkewMat(r);
    return X;
  }

  /*!
 * Get rotation matrix from spatial transformation
 */
  RotMat RotationFromSpatialXform(const SpatialXform &X)
  {
    RotMat R = X.topLeftCorner<3, 3>();
    return R;
  }

  /*!
 * Get translation vector from spatial transformation
 */
  Vector3d TranslationFromSpatialXform(const SpatialXform &X)
  {
    RotMat R = RotationFromSpatialXform(X);
    Vector3d r =
        -MatToSkewVec(R.transpose() * X.bottomLeftCorner<3, 3>());
    return r;
  }

  /*!
 * Invert a spatial transformation (much faster than matrix inverse)
 */
  SpatialXform InvertSpatialXform(const SpatialXform &X)
  {
    RotMat R = RotationFromSpatialXform(X);
    Vector3d r =
        -MatToSkewVec(R.transpose() * X.bottomLeftCorner<3, 3>());
    SpatialXform Xinv = CreateSpatialXform(R.transpose(), -R * r);
    return Xinv;
  }

  /*!
 * Compute joint motion subspace vector
 */
  SpatialVec JointMotionSubspace(JointType joint, CoordinateAxis axis)
  {
    Vector3d v(0, 0, 0);
    SpatialVec phi = SpatialVec::Zero();
    if (axis == CoordinateAxis::X)
      v(0) = 1;
    else if (axis == CoordinateAxis::Y)
      v(1) = 1;
    else
      v(2) = 1;

    if (joint == JointType::Prismatic)
      phi.bottomLeftCorner<3, 1>() = v;
    else if (joint == JointType::Revolute)
      phi.topLeftCorner<3, 1>() = v;
    else
      throw std::runtime_error("Unknown motion subspace");

    return phi;
  }

  /*!
 * Compute joint transformation
 */
  Matrix6d JointXform(JointType joint, CoordinateAxis axis, double q)
  {
    Matrix6d X = Matrix6d::Zero();
    if (joint == JointType::Revolute)
    {
      X = SpatialRotation(axis, q);
    }
    else if (joint == JointType::Prismatic)
    {
      Vector3d v(0, 0, 0);
      if (axis == CoordinateAxis::X)
        v(0) = q;
      else if (axis == CoordinateAxis::Y)
        v(1) = q;
      else if (axis == CoordinateAxis::Z)
        v(2) = q;

      X = CreateSpatialXform(RotMat::Identity(), v);
    }
    else
    {
      throw std::runtime_error("Unknown joint xform\n");
    }
    return X;
  }

  /*!
 * Construct the rotational inertia of a uniform density box with a given mass.
 * @param mass Mass of the box
 * @param dims Dimensions of the box
 */
  Matrix3d RotInertiaOfBox(double mass, const Vector3d &dims)
  {
    Matrix3d I =
        Matrix3d::Identity() * dims.norm() * dims.norm();
    for (size_t i = 0; i < 3; i++)
      I(i, i) -= dims(i) * dims(i);
    I = I * mass / 12;
    return I;
  }

  /*!
 * Convert from spatial velocity to linear velocity.
 * Uses spatial velocity at the given point.
 */
  Vector3d SpatialToLinearVelocity(const SpatialVec &v, const Vector3d &x)
  {
    Vector3d vsAng = v.topLeftCorner<3, 1>();
    Vector3d vsLin = v.bottomLeftCorner<3, 1>();
    Vector3d vLinear = vsLin + vsAng.cross(x);
    return vLinear;
  }

  /*!
 * Convert from spatial velocity to angular velocity.
 */
  Vector3d SpatialToAngularVelocity(const SpatialVec &v)
  {
    Vector3d vsAng = v.topLeftCorner<3, 1>();
    return vsAng;
  }

  /*!
 * Compute the classical lienear accleeration of a frame given its spatial
 * acceleration and velocity
 */
  Vector3d SpatialToLinearAcceleration(const SpatialVec &a, const SpatialVec &v)
  {
    Vector3d acc;
    // classical accleration = spatial linear acc + omega x v
    acc = a.tail<3>() + v.head<3>().cross(v.tail<3>());
    return acc;
  }

  /*!
 * Compute the classical lienear acceleration of a frame given its spatial
 * acceleration and velocity
 */
  Vector3d SpatialToLinearAcceleration(const SpatialVec &a, const SpatialVec &v, const Vector3d &x)
  {
    Vector3d alin_x = SpatialToLinearVelocity(a, x);
    Vector3d vlin_x = SpatialToLinearVelocity(v, x);

    // classical accleration = spatial linear acc + omega x v
    Vector3d acc = alin_x + v.head<3>().cross(vlin_x);
    return acc;
  }

  /*!
 * Apply spatial transformation to a point.
 */
  Vector3d SpatialXformPoint(const SpatialXform &X, const Vector3d &p)
  {
    Matrix3d R = RotationFromSpatialXform(X);
    Vector3d r = TranslationFromSpatialXform(X);
    Vector3d Xp = R * (p - r);
    return Xp;
  }

  /*!
 * Convert a force at a point to a spatial force
 * @param f : force
 * @param p : point
 */
  SpatialVec ForceToSpatialForce(const Vector3d &f, const Vector3d &p)
  {
    SpatialVec fs;
    fs.topLeftCorner<3, 1>() = p.cross(f);
    fs.bottomLeftCorner<3, 1>() = f;
    return fs;
  }

}
