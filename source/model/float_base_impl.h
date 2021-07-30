#pragma once

#include "sdrobot/model.h"
#include "dynamics/types.h"

namespace sdrobot::model
{
  class FloatBaseModelImpl : public FloatBaseModel
  {
  public:
    bool UpdateState(FloatBaseModelState const &state) override;
    FloatBaseModelState const &GetState() const override { return state_; }
    bool UpdateGravity(SdVector3f const &g) override;

    bool ComputeGeneralizedGravityForce() override;
    bool ComputeGeneralizedCoriolisForce() override;
    bool ComputeContactJacobians() override;

    SdMatrixXf const &GetMassMatrix() const override { return H_; }
    SdVectorXf const &GetGeneralizedGravityForce() const override { return G_; }
    SdVectorXf const &GetGeneralizedCoriolisForce() const override { return Cqd_; }
    std::vector<SdMatrixXf> const &GetContactJacobians() const override { return Jc_; }
    std::vector<SdVector3f> const &GetContactJacobiansdqd() const override { return Jcdqd_; }
    std::vector<SdVector3f> const &GetGroundContactPos() const override { return gc_p_; }
    std::vector<SdVector3f> const &GetGroundContactVel() const override { return gc_v_; }

    /*!
    * Create the floating body
    * @param inertia Spatial inertia of the floating body
    */
    bool AddBase(Eigen::Ref<dynamics::SpatialInertia const> const &inertia);

    /*!
    * Create the floating body
    * @param mass Mass of the floating body
    * @param com  Center of mass of the floating body
    * @param I    Rotational inertia of the floating body
    */
    bool AddBase(double const mass, Eigen::Ref<Vector3 const> const &com, dynamics::RotationalInertia const &I);

    /*!
    * Add a ground contact point to a model
    * @param bodyID The ID of the body containing the contact point
    * @param location The location (in body coordinate) of the contact point
    * @param isFoot True if foot or not.
    * @return The ID of the ground contact point
    */
    int AddGroundContactPoint(int const body_id, Eigen::Ref<Vector3 const> const &location,
                              bool const is_foot = false);

    /*!
    * Add the bounding points of a box to the contact model. Assumes the box is
    * centered around the origin of the body coordinate system and is axis aligned.
    */
    void AddGroundContactBoxPoints(int const body_id, Eigen::Ref<Vector3 const> const &dims);

    /*!
    * Add a body
    * @param inertia The inertia of the body
    * @param rotor_inertia The inertia of the rotor the body is connected to
    * @param gear_ratio The gear ratio between the body and the rotor
    * @param parent The parent body, which is also assumed to be the body the rotor
    * is connected to
    * @param joint_type The type of joint (prismatic or revolute)
    * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
    * @param Xtree  The coordinate transformation from parent to this body
    * @param Xrot  The coordinate transformation from parent to this body's rotor
    * @return The body's ID (can be used as the parent)
    */
    int AddBody(dynamics::SpatialInertia const &inertia,
                dynamics::SpatialInertia const &rotor_inertia, double const gear_ratio, int const parent,
                dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                Matrix6 const &Xtree, Matrix6 const &Xrot);

    /*!
    * Add a body
    * @param inertia The inertia of the body
    * @param rotor_inertia The inertia of the rotor the body is connected to
    * @param gear_ratio The gear ratio between the body and the rotor
    * @param parent The parent body, which is also assumed to be the body the rotor
    * is connected to
    * @param joint_type The type of joint (prismatic or revolute)
    * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
    * @param Xtree  The coordinate transformation from parent to this body
    * @param Xrot  The coordinate transformation from parent to this body's rotor
    * @return The body's ID (can be used as the parent)
    */
    int AddBody(dynamics::MassProperties const &inertia,
                dynamics::MassProperties const &rotor_inertia, double const gear_ratio, int const parent,
                dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                Matrix6 const &Xtree, Matrix6 const &Xrot);

  private:
    bool CompositeInertias();
    bool BiasAccelerations();
    bool ForwardKinematics();
    void ResetCalculationFlags();

    int n_dof_ = 0;
    SdVector3f gravity_;

    std::vector<int> parents_;

    FloatBaseModelState state_;

    std::vector<SdMatrix6f> Ibody_, Irot_;

    int n_ground_contact_ = 0;
    std::vector<int> gc_parent_;
    std::vector<SdVector3f> gc_location_;
    std::vector<SdVector3f> gc_p_;
    std::vector<SdVector3f> gc_v_;
    std::vector<bool> compute_contact_info_;

    std::vector<SdVector6f> v_, vrot_, S_, Srot_, ag_, agrot_, avp_, avprot_, fvp_, fvprot_;

    std::vector<SdMatrix6f> IC_;
    std::vector<SdMatrix6f> Xup_, Xuprot_, Xa_;

    SdMatrixXf H_;
    SdVectorXf Cqd_, G_;

    std::vector<SdMatrixXf> Jc_; //vector of matrix 3 x X
    std::vector<SdVector3f> Jcdqd_;
  };
}
