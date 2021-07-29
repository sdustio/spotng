#pragma once

#include <memory>

#include "types.h"

namespace sdrobot::model
{
  struct SDROBOT_EXPORT FloatBaseModelState
  {
    SdVector4f ori;
    SdMatrix3f pos;
    std::array<fptype, 6> vel_body; // body coordinates
    std::array<fptype, 12> q;
    std::array<fptype, 12> qd;
  };

  class SDROBOT_EXPORT FloatBaseModel
  {
  public:
    using Ptr = std::unique_ptr<FloatBaseModel>;
    using SharedPtr = std::shared_ptr<FloatBaseModel>;

    virtual ~FloatBaseModel() = default;

    /*!
    * Create the floating body
    * @param mass Mass of the floating body
    * @param com  Center of mass of the floating body
    * @param I    Rotational inertia of the floating body
    */
    virtual bool AddBase(double const mass, SdVector3f const &com, SdMatrix3f const &I) = 0;

    /*!
    * Add a ground contact point to a model
    * @param bodyID The ID of the body containing the contact point
    * @param location The location (in body coordinate) of the contact point
    * @param isFoot True if foot or not.
    * @return The ID of the ground contact point
    */
    virtual int AddGroundContactPoint(int const body_id, SdVector3f const &location,
                                 bool const is_foot = false) = 0;

    /*!
    * Add the bounding points of a box to the contact model. Assumes the box is
    * centered around the origin of the body coordinate system and is axis aligned.
    */
    virtual void AddGroundContactBoxPoints(int const body_id, SdVector3f const &dims) = 0;

    virtual bool UpdateState(FloatBaseModelState const &state) = 0;
    virtual FloatBaseModelState const &GetState() const = 0;
    virtual bool UpdateGravity(SdVector3f const &g) = 0;
    virtual SdVector3f const &GetGravity() const = 0;

    virtual bool ComputeGeneralizedGravityForce() const = 0;
    virtual bool ComputeGeneralizedCoriolisForce() const = 0;
    virtual bool ComputeContactJacobians() const = 0;

    virtual SdMatrixXf const &GetMassMatrix() const = 0;
    virtual SdVectorXf const &GetGeneralizedGravityForce() const = 0;
    virtual SdVectorXf const &GetGeneralizedCoriolisForce() const = 0;
    virtual std::vector<SdMatrixXf> const &GetContactJacobians() const = 0; //vector of matrix 3 x X
    virtual std::vector<SdMatrix3f> const &GetContactJacobiansdqd() const = 0;
    virtual std::vector<SdMatrix3f> const &GetGroundContactPos() const = 0;
    virtual std::vector<SdMatrix3f> const &GetGroundContactVel() const = 0;
  };

  class SDROBOT_EXPORT Quadruped
  {
  public:
    using Ptr = std::unique_ptr<Quadruped>;
    using SharedPtr = std::shared_ptr<Quadruped>;

    virtual ~Quadruped() = default;

    virtual bool ComputeFloatBaseModel(double g) = 0; // gravity
    virtual FloatBaseModel::SharedPtr const &GetFloatBaseModel() const = 0;

    virtual bool CalcHipLocation(SdVector3f &ret, int const leg) const = 0;
  };
} // namespace sdrobot::model
