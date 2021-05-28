#include "robot/model.h"
#include "dynamics/spatial.h"

using namespace sd::dynamics;

namespace sd::robot
{
  template <typename T>
  Quadruped<T>::Quadruped()
  {
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    InertiaMat<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = dynamics::CoordinateRot<T>(dynamics::CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = dynamics::CoordinateRot<T>(dynamics::CoordinateAxis::X, M_PI / 2);
    InertiaMat<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    InertiaMat<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vec3<T> rotorCOM(0, 0, 0);
    abad_rotor_spatial_inertia_ = BuildSpatialInertia<T>(0.055, rotorCOM, rotorRotationalInertiaX);
    knee_rotor_spatial_inertia_ = hip_rotor_spatial_inertia_ = BuildSpatialInertia<T>(0.055, rotorCOM, rotorRotationalInertiaY);

    // spatial inertias
    InertiaMat<T> abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0, 0.036, 0); // LEFT
    abad_spatial_inertia_ = BuildSpatialInertia<T>(0.54, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(0, 0.016, -0.02);
    hip_spatial_inertia_ = BuildSpatialInertia<T>(0.634, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0, 0, -0.061);
    knee_spatial_inertia_ = BuildSpatialInertia<T>(0.064, kneeCOM, kneeRotationalInertia);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(0, 0, 0);
    body_spatial_inertia_ = BuildSpatialInertia<T>(Properties::body_mass, bodyCOM,
                                                   bodyRotationalInertia);

    // locations
    abad_rotor_location_ = Vec3<T>(0.125, 0.049, 0);
    abad_location_ =
        Vec3<T>(Properties::body_length, Properties::body_width, 0) * 0.5;
    hip_location_ = Vec3<T>(0, Properties::abad_link_length, 0);
    hip_rotor_location_ = Vec3<T>(0, 0.04, 0);
    knee_location_ = Vec3<T>(0, 0, -Properties::hip_link_length);
    knee_rotor_location_ = Vec3<T>(0, 0, 0);
  }

  template <typename T>
  bool Quadruped<T>::BuildModel(dynamics::FBModel<T> &model)
  {
    // we assume the cheetah's body (not including rotors) can be modeled as a uniformly distributed box.
    //我们假设猎豹的身体(不包括转子)可以被建模为一个均匀分布的盒子。
    Vec3<T> bodyDims(Properties::body_length, Properties::body_width, Properties::body_height);

    // model.addBase(_bodyMass, Vec3<T>(0,0,0), RotInertiaOfBox(_bodyMass,
    // bodyDims));
    model.AddBase(body_spatial_inertia_);
    // add contact for the cheetah's body
    model.AddGroundContactBoxPoints(5, bodyDims);

    const int base_id = 5;
    int body_id = base_id;
    T side_sign = -1;

    Mat3<T> I3 = Mat3<T>::Identity();

    // loop over 4 legs
    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
      // Ab/Ad joint
      //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
      //  rotorInertia, T gearRatio,
      //              int parent, JointType jointType, CoordinateAxis jointAxis,
      //              const Mat6<T>& Xtree, const Mat6<T>& Xrot);
      body_id++;
      Mat6<T> xtree_abad = CreateSXform(I3, WithLegSigns(abad_location_, leg_id));
      Mat6<T> xtree_abad_rotor =
          CreateSXform(I3, WithLegSigns(abad_rotor_location_, leg_id));

      if (side_sign < 0)
      {
        model.AddBody(
            SpatialInertiaFlipAlongAxis(abad_spatial_inertia_, CoordinateAxis::Y),
            SpatialInertiaFlipAlongAxis(abad_rotor_spatial_inertia_, CoordinateAxis::Y),
            Properties::abad_gear_ratio, base_id, JointType::Revolute,
            CoordinateAxis::X, xtree_abad, xtree_abad_rotor);
      }
      else
      {
        model.AddBody(
            abad_spatial_inertia_, abad_rotor_spatial_inertia_, Properties::abad_gear_ratio,
            base_id, JointType::Revolute, CoordinateAxis::X, xtree_abad, xtree_abad_rotor);
      }

      // Hip Joint
      body_id++;
      Mat6<T> xtree_hip =
          CreateSXform(CoordinateRot<T>(CoordinateAxis::Z, T(M_PI)),
                       WithLegSigns(hip_location_, leg_id));
      Mat6<T> xtree_hip_rotor =
          CreateSXform(CoordinateRot<T>(CoordinateAxis::Z, T(M_PI)),
                       WithLegSigns(hip_rotor_location_, leg_id));

      if (side_sign < 0)
      {
        model.AddBody(
            SpatialInertiaFlipAlongAxis(hip_spatial_inertia_, CoordinateAxis::Y),
            SpatialInertiaFlipAlongAxis(hip_rotor_spatial_inertia_, CoordinateAxis::Y),
            Properties::hip_gear_ratio, body_id - 1, JointType::Revolute,
            CoordinateAxis::Y, xtree_hip, xtree_hip_rotor);
      }
      else
      {

        model.AddBody(
            hip_spatial_inertia_, hip_rotor_spatial_inertia_, Properties::hip_gear_ratio,
            body_id - 1, JointType::Revolute, CoordinateAxis::Y, xtree_hip, xtree_hip_rotor);
      }

      // add knee ground contact point
      model.AddGroundContactPoint(body_id, Vec3<T>(0, 0, -Properties::hip_link_length));

      // Knee Joint
      body_id++;
      Mat6<T> xtree_knee = CreateSXform(I3, knee_location_);
      Mat6<T> xtree_knee_rotor = CreateSXform(I3, knee_rotor_location_);
      if (side_sign < 0)
      {
        model.AddBody(
            SpatialInertiaFlipAlongAxis(knee_spatial_inertia_, CoordinateAxis::Y),
            SpatialInertiaFlipAlongAxis(knee_rotor_spatial_inertia_, CoordinateAxis::Y),
            Properties::knee_gear_ratio, body_id - 1, JointType::Revolute,
            CoordinateAxis::Y, xtree_knee, xtree_knee_rotor);

        model.AddGroundContactPoint(
            body_id, Vec3<T>(0, Properties::knee_link_y_offset, -Properties::knee_link_length), true);
      }
      else
      {

        model.AddBody(
            knee_spatial_inertia_, knee_rotor_spatial_inertia_, Properties::knee_gear_ratio,
            body_id - 1, JointType::Revolute, CoordinateAxis::Y, xtree_knee, xtree_knee_rotor);

        model.AddGroundContactPoint(
            body_id, Vec3<T>(0, -Properties::knee_link_y_offset, -Properties::knee_link_length), true);
      }

      // add foot
      //model.addGroundContactPoint(body_id, Vec3<T>(0, 0, -_kneeLinkLength), true);

      side_sign *= -1;
    }

    Vec3<T> g(0, 0, -9.81);
    model.SetGravity(g);

    return true;
  }

  template <typename T>
  void Quadruped<T>::UpdateLegData(const SPIData& data)
  {
    for (int leg = 0; leg < 4; leg++)
    {
      // q: 关节角
      leg_data[leg].q(0) = data.q_abad[leg];
      leg_data[leg].q(1) = data.q_hip[leg];
      leg_data[leg].q(2) = data.q_knee[leg];

      // qd 关节角速度？
      leg_data[leg].qd(0) = data.qd_abad[leg];
      leg_data[leg].qd(1) = data.qd_hip[leg];
      leg_data[leg].qd(2) = data.qd_knee[leg];

      // J and p 雅可比和足端位置
      ComputeLegJacobianAndPosition(leg);

      // v 足端速度
      leg_data[leg].v = leg_data[leg].J * leg_data[leg].qd;
    }
  }

  template <typename T>
  void Quadruped<T>::UpdateLegCmd(SPICmd &cmd)
  {
    for (int leg = 0; leg < 4; leg++)
    {
      // tauFF 获得从控制器来的力矩
      Vec3<T> legTorque = leg_cmd[leg].tau_feed_forward;

      // forceFF 获得从控制器来的力矩
      Vec3<T> footForce = leg_cmd[leg].force_feed_forward;

      // cartesian PD 直角坐标下pd
      footForce +=
          leg_cmd[leg].kp_cartesian * (leg_cmd[leg].p_des - leg_data[leg].p);
      footForce +=
          leg_cmd[leg].kd_cartesian * (leg_cmd[leg].v_des - leg_data[leg].v);

      // Torque 足力转换成力矩
      legTorque += leg_data[leg].J.transpose() * footForce;

      // set command: 命令设置 设置力矩
      cmd.tau_abad_ff[leg] = legTorque(0);
      cmd.tau_hip_ff[leg] = legTorque(1);
      cmd.tau_knee_ff[leg] = legTorque(2);

      // joint space pd
      // joint space PD
      cmd.kd_abad[leg] = leg_cmd[leg].kd_joint(0, 0);
      cmd.kd_hip[leg] = leg_cmd[leg].kd_joint(1, 1);
      cmd.kd_knee[leg] = leg_cmd[leg].kd_joint(2, 2);

      cmd.kp_abad[leg] = leg_cmd[leg].kp_joint(0, 0);
      cmd.kp_hip[leg] = leg_cmd[leg].kp_joint(1, 1);
      cmd.kp_knee[leg] = leg_cmd[leg].kp_joint(2, 2);

      cmd.q_des_abad[leg] = leg_cmd[leg].q_des(0);
      cmd.q_des_hip[leg] = leg_cmd[leg].q_des(1);
      cmd.q_des_knee[leg] = leg_cmd[leg].q_des(2);

      cmd.qd_des_abad[leg] = leg_cmd[leg].qd_des(0);
      cmd.qd_des_hip[leg] = leg_cmd[leg].qd_des(1);
      cmd.qd_des_knee[leg] = leg_cmd[leg].qd_des(2);

      // estimate torque
      leg_data[leg].tau_estimate =
          legTorque +
          leg_cmd[leg].kp_joint * (leg_cmd[leg].q_des - leg_data[leg].q) +
          leg_cmd[leg].kd_joint * (leg_cmd[leg].qd_des - leg_data[leg].qd);

      cmd.flags[leg] = legs_enabled ? 1 : 0;
    }
  }

  template <typename T>
  void Quadruped<T>::ComputeLegJacobianAndPosition(int leg)
  {
    T l1 = Properties::abad_link_length;
    T l2 = Properties::hip_link_length;
    T l3 = Properties::knee_link_length;
    T l4 = Properties::knee_link_y_offset;
    T side_sign = GetSideSign(leg);

    T s1 = std::sin(leg_data[leg].q(0));
    T s2 = std::sin(leg_data[leg].q(1));
    T s3 = std::sin(leg_data[leg].q(2));

    T c1 = std::cos(leg_data[leg].q(0));
    T c2 = std::cos(leg_data[leg].q(1));
    T c3 = std::cos(leg_data[leg].q(2));

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    leg_data[leg].J.operator()(0, 0) = 0;
    leg_data[leg].J.operator()(0, 1) = l3 * c23 + l2 * c2;
    leg_data[leg].J.operator()(0, 2) = l3 * c23;
    leg_data[leg].J.operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * side_sign * s1;
    leg_data[leg].J.operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    leg_data[leg].J.operator()(1, 2) = -l3 * s1 * s23;
    leg_data[leg].J.operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * side_sign * c1;
    leg_data[leg].J.operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    leg_data[leg].J.operator()(2, 2) = l3 * c1 * s23;

    leg_data[leg].p.operator()(0) = l3 * s23 + l2 * s2;
    leg_data[leg].p.operator()(1) = (l1 + l4) * side_sign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    leg_data[leg].p.operator()(2) = (l1 + l4) * side_sign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }

  template class Quadruped<double>;
  template class Quadruped<float>;
}
