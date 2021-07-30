#include "model/float_base_impl.h"
#include "dynamics/inertia.h"

namespace sdrobot::model
{
  bool FloatBaseModelImpl::UpdateState(FloatBaseModelState const &state)
  {
    state_ = state;
    ResetCalculationFlags();
  }

  bool FloatBaseModelImpl::UpdateGravity(SdVector3f const &g)
  {
    gravity_ = g;
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedGravityForce()
  {

    CompositeInertias();

    dynamics::SpatialVec aGravity;

    aGravity << 0, 0, 0, gravity_[0], gravity_[1], gravity_[2];
    ToEigenMatrix(ag_[5]) = ToConstEigenMatrix(Xup_[5]) * aGravity;

    // Gravity comp force is the same as force required to accelerate
    // oppostite gravity
    ToEigenMatrix(G_, n_dof_).topRows<6>() = -ToConstEigenMatrix(IC_[5]) * ToConstEigenMatrix(ag_[5]);
    for (int i = 6; i < n_dof_; i++)
    {
      ToEigenMatrix(ag_[i]) = ToConstEigenMatrix(Xup_[i]) * ToConstEigenMatrix(ag_[parents_[i]]);
      ToEigenMatrix(agrot_[i]) = ToConstEigenMatrix(Xuprot_[i]) * ToConstEigenMatrix(ag_[parents_[i]]);

      // body and rotor
      G_[i] = -ToConstEigenMatrix(S_[i]).dot(ToConstEigenMatrix(IC_[i]) * ToConstEigenMatrix(ag_[i])) -
              ToConstEigenMatrix(Srot_[i]).dot(ToConstEigenMatrix(Irot_[i]) * ToConstEigenMatrix(agrot_[i]));
    }
    return true;
  }

  bool FloatBaseModelImpl::ComputeGeneralizedCoriolisForce()
  {
    BiasAccelerations();

    dynamics::SpatialVec tmpsv;

    // Floating base force
    Matrix6 Ifb = ToConstEigenMatrix(Ibody_[5]);
    dynamics::ForceCrossProduct(tmpsv, ToConstEigenMatrix(v_[5]), Ifb * ToConstEigenMatrix(v_[5]));
    ToEigenMatrix(fvp_[5]) = Ifb * ToConstEigenMatrix(avp_[5]) + tmpsv;

    for (int i = 6; i < n_dof_; i++)
    {
      // Force on body i
      auto Ii = ToConstEigenMatrix(Ibody_[i]);
      auto vi = ToConstEigenMatrix(v_[i]);
      dynamics::ForceCrossProduct(tmpsv, vi, Ii * vi);
      ToEigenMatrix(fvp_[i]) = Ii * ToConstEigenMatrix(avp_[i]) + tmpsv;

      // Force on rotor i
      auto Ir = ToConstEigenMatrix(Irot_[i]);
      auto vr = ToConstEigenMatrix(vrot_[i]);
      dynamics::ForceCrossProduct(tmpsv, vr, Ir * vr);
      ToEigenMatrix(fvprot_[i]) = Ir * ToConstEigenMatrix(avprot_[i]) + tmpsv;
    }

    for (int i = n_dof_ - 1; i > 5; i--)
    {
      // Extract force along the joints
      Cqd_[i] = ToConstEigenMatrix(S_[i]).dot(ToConstEigenMatrix(fvp_[i])) + ToConstEigenMatrix(Srot_[i]).dot(ToConstEigenMatrix(fvprot_[i]));

      // Propage force down the tree
      ToEigenMatrix(fvp_[parents_[i]]) += ToConstEigenMatrix(Xup_[i]).transpose() * ToConstEigenMatrix(fvp_[i]);
      ToEigenMatrix(fvp_[parents_[i]]) += ToConstEigenMatrix(Xuprot_[i]).transpose() * ToConstEigenMatrix(fvprot_[i]);
    }

    // Force on floating base
    ToEigenMatrix(Cqd_, n_dof_).topRows<6>() = ToConstEigenMatrix(fvp_[5]);
    return true;
  }

  bool FloatBaseModelImpl::ComputeContactJacobians() {}

  bool FloatBaseModelImpl::AddBase(Eigen::Ref<dynamics::SpatialInertia const> const &inertia) {}
  bool FloatBaseModelImpl::AddBase(double const mass, Eigen::Ref<Vector3 const> const &com, dynamics::RotationalInertia const &I) {}
  int FloatBaseModelImpl::AddGroundContactPoint(int const body_id, Eigen::Ref<Vector3 const> const &location,
                                                bool const is_foot) {}

  void FloatBaseModelImpl::AddGroundContactBoxPoints(int const body_id, Eigen::Ref<Vector3 const> const &dims) {}
  int FloatBaseModelImpl::AddBody(dynamics::SpatialInertia const &inertia,
                                  dynamics::SpatialInertia const &rotor_inertia, double const gear_ratio, int const parent,
                                  dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                                  Matrix6 const &Xtree, Matrix6 const &Xrot) {}
  int FloatBaseModelImpl::AddBody(dynamics::MassProperties const &inertia,
                                  dynamics::MassProperties const &rotor_inertia, double const gear_ratio, int const parent,
                                  dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                                  Matrix6 const &Xtree, Matrix6 const &Xrot) {}

}
