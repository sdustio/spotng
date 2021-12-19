#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dynamics/types.h"
#include "sdquadx/model.h"

namespace sdquadx::model {

struct FBModelState {
  SdMatrix3f rot_mat;  // 从世界坐标到机身坐标
  SdVector3f pos;
  SdVector6f gvel_robot;  // combine of avel and lvel [avel, lvel]
  std::array<fpt_t, consts::model::kNumJoint> q;
  std::array<fpt_t, consts::model::kNumJoint> qd;
};

class FBModel {
 public:
  using Ptr = std::unique_ptr<FBModel>;
  using SharedPtr = std::shared_ptr<FBModel>;
  using ConstSharedPtr = std::shared_ptr<FBModel const>;

  FBModelState &GetStateForUpdate();
  bool UpdateGravity(SdVector3f const &g);

  bool ComputeGeneralizedMassMatrix(DynamicsData &ret);
  bool ComputeGeneralizedGravityForce(DynamicsData &ret);
  bool ComputeGeneralizedCoriolisForce(DynamicsData &ret);
  bool ComputeContactJacobians(DynamicsData &ret);

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
  bool AddBase(fpt_t const mass, Eigen::Ref<Vector3 const> const &com, dynamics::InertiaTensor const &I);

  /*!
   * Add a foot to a model
   * @param bodyID The ID of the body containing the contact point
   * @param location The location (in body coordinate) of the contact point
   * @param isFoot True if foot or not.
   * @return The ID of the ground contact point
   */
  int AddFoot(int const body_id, Eigen::Ref<Vector3 const> const &location);

  /*!
   * Add a body
   * @param inertia The inertia of the body
   * @param parent The parent body, which is also assumed to be the body the
   * rotor is connected to
   * @param joint_type The type of joint (prismatic or revolute)
   * @param joint_axis The joint axis (X,Y,Z), in the parent's frame
   * @param Xtree  The coordinate transformation from parent to this body
   * @return The body's ID (can be used as the parent)
   */
  int AddBody(Eigen::Ref<dynamics::SpatialInertia const> const &inertia, int const parent,
              dynamics::JointType const joint_type, dynamics::CoordinateAxis const joint_axis,
              Eigen::Ref<Matrix6 const> const &Xtree);

 private:
  /*!
   * Populate member variables when bodies are added
   * @param count (6 for fb, 1 for joint)
   */
  bool AddDynamicsVars(int count);

  bool CompositeInertias();
  bool BiasAccelerations();
  bool ForwardKinematics();
  bool ResetCalculationFlags();

  int curr_n_dof_ = 0;
  SdVector3f gravity_;
  FBModelState state_;

  std::vector<int> parents_;
  std::vector<dynamics::JointType> joint_types_;
  std::vector<dynamics::CoordinateAxis> joint_axes_;
  std::vector<SdMatrix6f> Xtree_;
  std::vector<SdMatrix6f> Ibody_;
  std::vector<std::string> body_names_;

  int foot_count_ = 0;
  std::vector<int> gc_parent_;
  std::vector<SdVector3f> gc_location_;

  std::vector<SdVector6f> v_, S_, ag_, avp_, fvp_, c_;

  std::vector<SdMatrix6f> IC_;
  std::vector<SdMatrix6f> Xup_, Xa_;

  bool kinematics_uptodate_ = false;
  bool bias_acc_uptodate_ = false;
  bool composite_inertias_uptodate_ = false;
};
}  // namespace sdquadx::model
