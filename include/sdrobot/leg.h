#pragma once

#include <memory>

#include "sdrobot/types.h"
#include "sdrobot/interface.h"

namespace sdrobot::leg
{
  namespace idx
  {
    constexpr inline int const fr = 0; // Front Right
    constexpr inline int const fl = 1; // Front Left
    constexpr inline int const hr = 2; // Hind Right
    constexpr inline int const hl = 3; // Hind Left
  }

  /*!
    * Data sent from the control algorithm to the legs.
    * 数据从控制算法发送到腿部。
    */
  struct SDROBOT_EXPORT Cmd
  {
    SdVector3f tau_feed_forward = {};
    SdVector3f q_des = {};
    SdVector3f qd_des = {};
    SdMatrix3f kp_joint = {};
    SdMatrix3f kd_joint = {};

    //from mpc; aid coumpter above;
    SdVector3f force_feed_forward = {};
    SdVector3f p_des = {};
    SdVector3f v_des = {};

    //from mpc; aid coumpter above;
    SdMatrix3f kp_cartesian = {};
    SdMatrix3f kd_cartesian = {};

    void Zero();
  };

  /*!
    * Data returned from the legs to the control code.
    * 从腿返回到控制代码的数据
    */
  struct SDROBOT_EXPORT Data
  {
    SdVector3f q = {};            //关节角度
    SdVector3f qd = {};           //关节角速度
    SdVector3f p = {};            //足端位置
    SdVector3f v = {};            //足端速度
    SdMatrix3f J = {};           //雅可比
    SdVector3f tau_estimate = {}; //估计力矩反馈

    void Zero();
  };

  using Cmds = std::array<Cmd, 4>;
  using Datas = std::array<Data, 4>;

  class SDROBOT_EXPORT LegCtrl
  {
  public:
    using Ptr = std::unique_ptr<LegCtrl>;
    using SharedPtr = std::shared_ptr<LegCtrl>;
    using ConstPtr = std::unique_ptr<LegCtrl const>;
    using ConstSharedPtr = std::shared_ptr<LegCtrl const>;

    virtual ~LegCtrl() = default;

    virtual const Datas &GetDatas() const = 0;

    virtual Cmds &GetCmdsForUpdate() = 0;
    virtual bool UpdateCmds(Cmds const &cmds) = 0;
    virtual bool UpdateCmd(int leg, Cmd const &cmd) = 0;

    /*!
    * Zero all leg commands.  This should be run *before* any control code, so if
    * the control code is confused and doesn't change the leg command, the legs
    * won't remember the last command.
    * 腿部控制命令清零，应运行在任何控制代码之前，否则控制代码混乱，控制命令不会改变，腿部不会记忆上次命令
    */
    virtual void ZeroCmd() = 0;

    /*!
    * Compute the position of the foot and its Jacobian.  This is done in the local
    * leg coordinate system. If J/p are NULL, the calculation will be skipped.
    */
    virtual void ComputeLegJacobianAndPosition(int leg) = 0;

    /*!
    * Update the "leg data" from Actuator Interface
    * 从spine卡 更新腿部信息
    */
    virtual void UpdateDatasFromActuatorInterface() = 0;

    /*!
    * Send the "leg command" to Actuator Interface
    * 向控制器发送控制命令
    */
    virtual void SendCmdsToActuatorInterface() = 0;
  };

  class SDROBOT_EXPORT JPosInit
  {
  public:
    using Ptr = std::unique_ptr<JPosInit>;
    using SharedPtr = std::shared_ptr<JPosInit>;
    using ConstPtr = std::unique_ptr<JPosInit const>;
    using ConstSharedPtr = std::shared_ptr<JPosInit const>;

    virtual ~JPosInit() = default;

    virtual bool IsInitialized(LegCtrl::SharedPtr &ctrl) = 0;
  };

} // namespace sdrobot::leg
