#pragma once

#include "sdrobot/model.h"
#include "dynamics/types.h"

namespace sdrobot::model
{
  class FloatBaseModelImpl : public FloatBaseModel
  {
  public:
    bool UpdateState(FloatBaseModelState const &state) override;
    FloatBaseModelState const &GetState() const override;
    bool UpdateGravity(SdVector3f const &g) override;
    SdVector3f const &GetGravity() const override;

    bool ComputeGeneralizedGravityForce() const override;
    bool ComputeGeneralizedCoriolisForce() const override;
    bool ComputeContactJacobians() const override;

    SdMatrixXf const &GetMassMatrix() const override;
    SdVectorXf const &GetGeneralizedGravityForce() const override;
    SdVectorXf const &GetGeneralizedCoriolisForce() const override;
    std::vector<SdMatrixXf> const &GetContactJacobians() const override; //vector of matrix 3 x X
    std::vector<SdMatrix3f> const &GetContactJacobiansdqd() const override;
    std::vector<SdMatrix3f> const &GetGroundContactPos() const override;
    std::vector<SdMatrix3f> const &GetGroundContactVel() const override;

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
    int AddBody(MassProperties const &inertia,
                MassProperties const &rotor_inertia, double const gear_ratio, int const parent,
                dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
                Matrix6 const &Xtree, Matrix6 const &Xrot);
  };
}
